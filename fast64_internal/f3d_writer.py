import bpy
import bmesh
import mathutils
from math import pi
from io import BytesIO
import os

import copy
from math import pi, ceil
from .utility import *
from .sm64_constants import *
from .f3d_material import all_combiner_uses
from .f3d_gbi import *
from .f3d_gbi import _DPLoadTextureBlock

bitSizeDict = {
	'G_IM_SIZ_4b' : 4,
	'G_IM_SIZ_8b' : 8,
	'G_IM_SIZ_16b' : 16,
	'G_IM_SIZ_32b' : 32,
}

texBitSizeOf = {
	'I4' : 'G_IM_SIZ_4b',
	'IA4' : 'G_IM_SIZ_4b',
	'CI4' : 'G_IM_SIZ_4b',
	'I8' : 'G_IM_SIZ_8b',
	'IA8' : 'G_IM_SIZ_8b',
	'CI8' : 'G_IM_SIZ_8b',
	'RGBA16' : 'G_IM_SIZ_16b',
	'IA16' : 'G_IM_SIZ_16b',
	'YUV16' : 'G_IM_SIZ_16b',
	'RGBA32' : 'G_IM_SIZ_32b',
}

texFormatOf = {
	'I4' : 'G_IM_FMT_I',
	'IA4' : 'G_IM_FMT_IA',
	'CI4' : 'G_IM_FMT_CI',
	'I8' : 'G_IM_FMT_I',
	'IA8' : 'G_IM_FMT_IA',
	'CI8' : 'G_IM_FMT_CI',
	'RGBA16' : 'G_IM_FMT_RGBA',
	'IA16' : 'G_IM_FMT_IA',
	'YUV16' : 'G_IM_FMT_YUV',
	'RGBA32' : 'G_IM_FMT_RGBA',
}

def getEdgeToFaceDict(mesh):
	edgeDict = {}
	for face in mesh.loop_triangles:
		for edgeKey in face.edge_keys:
			if edgeKey not in edgeDict:
				edgeDict[edgeKey] = []
			if face not in edgeDict[edgeKey]:
				edgeDict[edgeKey].append(face)
	return edgeDict

def getVertToFaceDict(mesh):
	vertDict = {}
	for face in mesh.loop_triangles:
		for vertIndex in face.vertices:
			if vertIndex not in vertDict:
				vertDict[vertIndex] = []
			if face not in vertDict[vertIndex]:
				vertDict[vertIndex].append(face)
	return vertDict

# Make sure to set original_name before calling this
# used when duplicating an object
def saveStaticModel(fModel, obj, transformMatrix):
	if len(obj.data.polygons) == 0:
		return None

	obj.data.calc_loop_triangles()
	obj.data.calc_normals_split()
	edgeDict = getEdgeToFaceDict(obj.data)

	fMeshGroup = FMeshGroup(toAlnum(obj.original_name), 
		FMesh(toAlnum(obj.original_name) + '_mesh'), None)
	fModel.meshGroups[obj.original_name] = fMeshGroup

	for material_index in range(len(obj.data.materials)):
		material = obj.data.materials[material_index]
		saveMeshByFaces(material, 
			[face for face in obj.data.loop_triangles if \
				face.material_index == material_index], 
			fModel, fMeshGroup.mesh, obj, transformMatrix, edgeDict)
	
	revertMatAndEndDraw(fMeshGroup.mesh.draw)
	return fMeshGroup

def cleanupDuplicatedObjects(selected_objects):
	meshData = []
	for selectedObj in selected_objects:
		meshData.append(selectedObj.data)
	for selectedObj in selected_objects:
		bpy.data.objects.remove(selectedObj)
	for mesh in meshData:
		bpy.data.meshes.remove(mesh)

def exportF3DCommon(obj, f3dType, isHWv1, transformMatrix):
	checkForF3DMaterial(obj)
	obj.original_name = obj.name
	bpy.ops.object.select_all(action = 'DESELECT')
	obj.select_set(True)
	bpy.context.view_layer.objects.active = obj
	bpy.ops.object.duplicate()
	fModel = FModel(f3dType, isHWv1)
	try:
		# duplicate obj and apply modifiers / make single user
		tempObj = bpy.context.selected_objects[0]
		bpy.ops.object.make_single_user(obdata = True)
		bpy.ops.object.transform_apply(location = False, 
			rotation = False, scale = True, properties =  False)
		for modifier in tempObj.modifiers:
			bpy.ops.object.modifier_apply(apply_as='DATA',
				modifier=modifier.name)
		fMeshGroup = saveStaticModel(fModel, tempObj, transformMatrix)
		cleanupDuplicatedObjects([tempObj])
		obj.select_set(True)
		bpy.context.view_layer.objects.active = obj
	except Exception as e:
		cleanupDuplicatedObjects([tempObj])
		raise Exception(str(e))
		obj.select_set(True)
		bpy.context.view_layer.objects.active = obj

	return fModel, fMeshGroup

def exportF3DtoC(dirPath, obj, isStatic, transformMatrix, 
	f3dType, isHWv1, texDir, savePNG, texSeparate):
	fModel, fMeshGroup = exportF3DCommon(obj, f3dType, isHWv1, transformMatrix)

	modelDirPath = os.path.join(dirPath, toAlnum(obj.name))

	if not os.path.exists(modelDirPath):
		os.mkdir(modelDirPath)

	if savePNG:
		fModel.save_c_tex_separate(isStatic, texDir, modelDirPath, texSeparate)
		fModel.freePalettes()
	else:
		fModel.freePalettes()
		modelPath = os.path.join(modelDirPath, 'model.inc.c')
	
		data = fModel.to_c(isStatic)
		outFile = open(modelPath, 'w')
		outFile.write(data)
		outFile.close()

	headerPath = os.path.join(modelDirPath, 'header.h')
	cDefine = fModel.to_c_def()
	cDefFile = open(headerPath, 'w')
	cDefFile.write(cDefine)
	cDefFile.close()

	if bpy.context.mode != 'OBJECT':
		bpy.ops.object.mode_set(mode = 'OBJECT')

def exportF3DtoBinary(romfile, exportRange, transformMatrix, 
	obj, f3dType, isHWv1, segmentData):
	fModel, fMeshGroup = exportF3DCommon(obj, f3dType, isHWv1, 
		transformMatrix)
	fModel.freePalettes()

	addrRange = fModel.set_addr(exportRange[0])
	if addrRange[1] > exportRange[1]:
		raise ValueError('Size too big: Data ends at ' + hex(addrRange[1]) +\
			', which is larger than the specified range.')
	fModel.save_binary(romfile, segmentData)
	if bpy.context.mode != 'OBJECT':
		bpy.ops.object.mode_set(mode = 'OBJECT')

	segPointerData = encodeSegmentedAddr(
		fMeshGroup.mesh.draw.startAddress, segmentData)

	return fMeshGroup.mesh.draw.startAddress, addrRange, segPointerData

def exportF3DtoBinaryBank0(romfile, exportRange, transformMatrix, 
	obj, f3dType, isHWv1, RAMAddr):
	fModel, fMeshGroup = exportF3DCommon(obj, f3dType, isHWv1, transformMatrix)
	fModel.freePalettes()
	segmentData = copy.copy(bank0Segment)

	addrRange = fModel.set_addr(RAMAddr)
	if addrRange[1] - RAMAddr > exportRange[1] - exportRange[0]:
		raise ValueError('Size too big: Data ends at ' + hex(addrRange[1]) +\
			', which is larger than the specified range.')

	bytesIO = BytesIO()
	#actualRAMAddr = get64bitAlignedAddr(RAMAddr)
	bytesIO.seek(RAMAddr)
	fModel.save_binary(bytesIO, segmentData)
	data = bytesIO.getvalue()[RAMAddr:]
	bytesIO.close()

	startAddress = get64bitAlignedAddr(exportRange[0])
	romfile.seek(startAddress)
	romfile.write(data)

	if bpy.context.mode != 'OBJECT':
		bpy.ops.object.mode_set(mode = 'OBJECT')

	segPointerData = encodeSegmentedAddr(
		fMeshGroup.mesh.draw.startAddress, segmentData)

	return (fMeshGroup.mesh.draw.startAddress, \
		(startAddress, startAddress + len(data)), segPointerData)

def checkForF3DMaterial(obj):
	if len(obj.material_slots) == 0:
		raise ValueError(obj.name + " has no Fast3D material.")
	for materialSlot in obj.material_slots:
		if materialSlot.material is None or \
			not materialSlot.material.is_f3d:
			raise ValueError(obj.name + " has either empty material slots " +\
				'or non-Fast3D materials.')

def revertMatAndEndDraw(gfxList):
	gfxList.commands.extend([
		DPPipeSync(),
		SPSetGeometryMode(['G_LIGHTING']),
		SPClearGeometryMode(['G_TEXTURE_GEN']),
		DPSetCombineMode(*S_SHADED_SOLID),
		SPTexture(0xFFFF, 0xFFFF, 0, 0, 0),
		SPEndDisplayList()])

def getCommonEdge(face1, face2, mesh):
	for edgeKey1 in face1.edge_keys:
		for edgeKey2 in face2.edge_keys:
			if edgeKey1 == edgeKey2:
				return edgeKey1
	raise ValueError("No common edge between faces " + str(face1.index) + \
		' and ' + str(face2.index))

def edgeValid(face1, face2, convertInfo, mesh):
	edgeKey = getCommonEdge(face1, face2, mesh)

	vertPairs = [[],[]]

	for loopIndex in face1.loops:
		vertIndex = mesh.loops[loopIndex].vertex_index
		if vertIndex in edgeKey:
			index = 0 if vertIndex == edgeKey[0] else 1
			vertPairs[index].append([loopIndex, face1])

	for loopIndex in face2.loops:
		vertIndex = mesh.loops[loopIndex].vertex_index
		if vertIndex in edgeKey:
			index = 0 if vertIndex == edgeKey[0] else 1
			vertPairs[index].append([loopIndex, face2])

	vertPairs[0] = [(mesh.loops[i], face) for i, face in vertPairs[0]]
	vertPairs[1] = [(mesh.loops[i], face) for i, face in vertPairs[1]]

	for vertPair in vertPairs:
		if getF3DVert(vertPair[0][0], vertPair[0][1], convertInfo, mesh) != \
			getF3DVert(vertPair[1][0], vertPair[1][1], convertInfo, mesh):
			return False

	return True

def getNumUnvisitedNeighbors(face, faces, visitedNeighbors, convertInfo,
	edgeDict, mesh):
	neighbors = []

	for edgeKey in face.edge_keys:
		for linkedFace in edgeDict[edgeKey]:
			if linkedFace.index != face.index:
				if linkedFace in faces and linkedFace not in neighbors and\
					linkedFace not in visitedNeighbors and \
					edgeValid(face, linkedFace, convertInfo, mesh):
					neighbors.append(linkedFace)
	return len(neighbors)

def getLowestUnvisitedNeighborCountFace(faces, visitedFaces, convertInfo, 
	edgeDict, mesh):
	lowestNeighborFace = None
	for face in faces:
		if face not in visitedFaces:
			lowestNeighborFace = face
			break
	lowestNeighborCount = getNumUnvisitedNeighbors(
		lowestNeighborFace, faces, visitedFaces, convertInfo, edgeDict, mesh)
	for face in faces:
		if face in visitedFaces:
			continue
		neighborCount = getNumUnvisitedNeighbors(
			face, faces, visitedFaces, convertInfo, edgeDict, mesh)
		if neighborCount < lowestNeighborCount:
			lowestNeighborFace = face
			lowestNeighborCount = neighborCount
	return lowestNeighborFace

def getNextNeighborFace(faces, face, lastEdgeKey, visitedFaces, possibleFaces,
	convertInfo, mesh, edgeDict):
	
	if lastEdgeKey is not None:
		handledEdgeKeys = [lastEdgeKey]
		nextEdgeKey = face.edge_keys[
			(face.edge_keys.index(lastEdgeKey) + 1) % 3]
	else:
		handledEdgeKeys = []
		nextEdgeKey = face.edge_keys[0]

	nextFaceAndEdge = (None, None)
	while nextEdgeKey not in handledEdgeKeys:
		for linkedFace in edgeDict[nextEdgeKey]:
			if linkedFace == face or linkedFace not in faces:
				continue
			elif edgeValid(linkedFace, face, convertInfo, mesh) and \
				linkedFace not in visitedFaces:
				if nextFaceAndEdge[0] is None:
					#print(nextLoop.face)
					nextFaceAndEdge = (linkedFace, nextEdgeKey)
				else:
					# Move face to front of queue
					if linkedFace in possibleFaces:
						possibleFaces.remove(linkedFace)
					possibleFaces.insert(0, linkedFace)
		handledEdgeKeys.append(nextEdgeKey)
		nextEdgeKey = face.edge_keys[
			(face.edge_keys.index(nextEdgeKey) + 1) % 3]
	return nextFaceAndEdge

def saveTriangleStrip(faces, convertInfo, triList, vtxList, f3d, 
	texDimensions, transformMatrix, isPointSampled, exportVertexColors,
	existingVertexData, existingVertexMaterialRegions, edgeDict, mesh):
	visitedFaces = []
	possibleFaces = []
	lastEdgeKey = None
	neighborFace = getLowestUnvisitedNeighborCountFace(
		faces, visitedFaces, convertInfo, edgeDict, mesh)

	triConverter = TriangleConverter(mesh, convertInfo, triList, vtxList, f3d, 
		texDimensions, transformMatrix, isPointSampled, exportVertexColors,
		existingVertexData, existingVertexMaterialRegions)

	while len(visitedFaces) < len(faces):
		#print(str(len(visitedFaces)) + " " + str(len(bFaces)))
		if neighborFace is None:
			if len(possibleFaces) > 0:
				#print("get neighbor from queue")
				neighborFace = possibleFaces[0]
				lastEdgeKey = None
				possibleFaces = []
			else:
				#print('get new neighbor')
				neighborFace =  getLowestUnvisitedNeighborCountFace(
					faces, visitedFaces, convertInfo, edgeDict, mesh)
				lastEdgeKey = None
		
		triConverter.addFace(neighborFace)
		if neighborFace in visitedFaces:
			raise ValueError("Repeated face")
		visitedFaces.append(neighborFace)
		if neighborFace in possibleFaces:
			possibleFaces.remove(neighborFace)

		neighborFace, lastEdgeKey = getNextNeighborFace(faces, 
			neighborFace, lastEdgeKey, visitedFaces, possibleFaces, convertInfo,
			mesh, edgeDict)
	
	triConverter.finish()

# Necessary for UV half pixel offset (see 13.7.5.3)
def isTexturePointSampled(material):
	return material.rdp_settings.g_mdsft_text_filt == 'G_TF_POINT'

def isLightingDisabled(material):
	return not material.rdp_settings.g_lighting

# Necessary as G_SHADE_SMOOTH actually does nothing
def checkIfFlatShaded(material):
	return not material.rdp_settings.g_shade_smooth

def saveMeshByFaces(material, faces, fModel, fMesh, obj, transformMatrix,
	edgeDict):
	if len(faces) == 0:
		print('0 Faces Provided.')
		return
	fMaterial, texDimensions = \
		saveOrGetF3DMaterial(material, fModel, obj)
	isPointSampled = isTexturePointSampled(material)
	exportVertexColors = isLightingDisabled(material)
	uv_layer = obj.data.uv_layers.active.data
	convertInfo = LoopConvertInfo(uv_layer, obj, exportVertexColors)

	fMesh.draw.commands.append(SPDisplayList(fMaterial.material))
	triList = fMesh.tri_list_new()
	fMesh.draw.commands.append(SPDisplayList(triList))

	#saveGeometry(obj, triList, fMesh.vertexList, bFaces, 
	#	bMesh, texDimensions, transformMatrix, isPointSampled, isFlatShaded,
	#	exportVertexColors, fModel.f3d)
	saveTriangleStrip(faces, convertInfo, triList, fMesh.vertexList,
		fModel.f3d, texDimensions, transformMatrix, isPointSampled,
		exportVertexColors, None, None, edgeDict, obj.data)
	
	if fMaterial.revert is not None:
		fMesh.draw.commands.append(SPDisplayList(fMaterial.revert))

def get8bitRoundedNormal(normal):
	return mathutils.Vector(
		(round(normal[0] * 128) / 128,
		round(normal[1] * 128) / 128,
		round(normal[2] * 128) / 128)
	)

class LoopConvertInfo:
	def __init__(self, uv_layer, obj, exportVertexColors):
		self.uv_layer = uv_layer
		self.obj = obj
		self.exportVertexColors = exportVertexColors

def getNewIndices(existingIndices, bufferStart):
	n = bufferStart
	newIndices = []
	for index in existingIndices:
		if index is None:
			newIndices.append(n)
			n += 1
		else:
			newIndices.append(index)
	return newIndices

class TriangleConverter:
	def __init__(self, mesh, convertInfo, triList, vtxList, f3d, 
		texDimensions, transformMatrix, isPointSampled, exportVertexColors,
		existingVertexData, existingVertexMaterialRegions):
		self.convertInfo = convertInfo
		self.mesh = mesh

		# Existing data assumed to be already loaded in.
		if existingVertexData is not None:
			# [(position, uv, colorOrNormal)]
			self.vertBuffer = existingVertexData 
		else:
			self.vertBuffer = []
		self.existingVertexMaterialRegions = existingVertexMaterialRegions
		self.bufferStart = len(self.vertBuffer)
		self.vertexBufferTriangles = [] # [(index0, index1, index2)]
		self.f3d = f3d
		self.triList = triList
		self.vtxList = vtxList

		self.texDimensions = texDimensions
		self.transformMatrix = transformMatrix
		self.isPointSampled = isPointSampled
		self.exportVertexColors = exportVertexColors

	def vertInBuffer(self, f3dVert, material_index):
		if self.existingVertexMaterialRegions is None:
			if f3dVert in self.vertBuffer:
				return self.vertBuffer.index(f3dVert)
			else:
				return None
		else:
			if material_index in self.existingVertexMaterialRegions:
				matRegion = self.existingVertexMaterialRegions[material_index]
				for i in range(matRegion[0], matRegion[1]):
					if self.vertBuffer[i] == f3dVert:
						return i
			for i in range(self.bufferStart, len(self.vertBuffer)): 
				if self.vertBuffer[i] == f3dVert:
					return i
			return None

	def addFace(self, face):
		triIndices = []
		existingVertIndices = []
		addedVerts = [] # verts added to existing vertexBuffer
		allVerts = [] # all verts not in 'untouched' buffer region

		for loopIndex in face.loops:
			loop = self.mesh.loops[loopIndex]
			f3dVert = getF3DVert(loop, face, self.convertInfo, self.mesh)
			vertIndex = self.vertInBuffer(f3dVert, face.material_index)
			if vertIndex is not None:
				triIndices.append(vertIndex)			
			else:
				addedVerts.append(f3dVert)
				triIndices.append(len(self.vertBuffer) + len(addedVerts) - 1)

			if f3dVert in self.vertBuffer[:self.bufferStart]:
				existingVertIndices.append(self.vertBuffer.index(f3dVert))
			else:
				allVerts.append(f3dVert)
				existingVertIndices.append(None)
		
		# We care only about load size, since loading is what takes up time.
		# Even if vert_buffer is larger, its still another load to fill it.
		if len(self.vertBuffer) + len(addedVerts) > self.f3d.vert_load_size:
			self.triList.commands.append(
				SPVertex(self.vtxList, len(self.vtxList.vertices), 
				len(self.vertBuffer) - self.bufferStart, self.bufferStart))
			self.triList.commands.extend(createTriangleCommands(
				self.vertexBufferTriangles, self.f3d.F3DEX_GBI))
			for vertData in self.vertBuffer[self.bufferStart:]:
				self.vtxList.vertices.append(convertVertexData(self.convertInfo.obj.data, 
					vertData[0], vertData[1], vertData[2], self.texDimensions,
					self.transformMatrix, self.isPointSampled,
					self.exportVertexColors))
			self.vertBuffer = self.vertBuffer[:self.bufferStart] + allVerts
			self.vertexBufferTriangles = \
				[getNewIndices(existingVertIndices, self.bufferStart)]
		else:
			self.vertBuffer.extend(addedVerts)
			self.vertexBufferTriangles.append(triIndices)
	
	def finish(self):
		if len(self.vertexBufferTriangles) > 0:
			self.triList.commands.append(SPVertex(self.vtxList, 
				len(self.vtxList.vertices), 
				len(self.vertBuffer) - self.bufferStart, self.bufferStart))
			self.triList.commands.extend(createTriangleCommands(
				self.vertexBufferTriangles, self.f3d.F3DEX_GBI))
			for vertData in self.vertBuffer[self.bufferStart:]:
				self.vtxList.vertices.append(convertVertexData(
					self.convertInfo.obj.data, vertData[0], vertData[1], 
					vertData[2], self.texDimensions, self.transformMatrix,
					self.isPointSampled, self.exportVertexColors))
		
		self.triList.commands.append(SPEndDisplayList())
	
def getF3DVert(loop, face, convertInfo, mesh):
	position = mesh.vertices[loop.vertex_index].co.copy().freeze()
	uv = convertInfo.uv_layer[loop.index].uv.copy().freeze()
	colorOrNormal = getLoopColorOrNormal(loop, face, 
		convertInfo.obj.data, convertInfo.obj, convertInfo.exportVertexColors)
	
	return (position, uv, colorOrNormal)

def getLoopNormal(loop, face, isFlatShaded):
	if isFlatShaded:
		normal = -face.normal #???
	else:
		normal = -loop.normal #???
	return get8bitRoundedNormal(normal).freeze()

'''
def getLoopNormalCreased(bLoop, obj):
	edges = obj.data.edges
	centerVert = bLoop.vert

	availableFaces = []
	visitedFaces = [bLoop.face]
	connectedFaces = getConnectedFaces(bLoop.face, bLoop.vert)
	if len(connectedFaces) == 0:
		return bLoop.calc_normal()

	for face in connectedFaces:
		availableFaces.append(FaceWeight(face, bLoop.face, 1))

	curNormal = bLoop.calc_normal() * bLoop.calc_angle()
	while len(availableFaces) > 0:
		nextFaceWeight = getHighestFaceWeight(availableFaces)
		curNormal += getWeightedNormalAndMoveToNextFace(
			nextFaceWeight, visitedFaces, availableFaces, centerVert, edges)
	
	return curNormal.normalized()

def getConnectedFaces(bFace, bVert):
	connectedFaces = []
	for face in bVert.link_faces:
		if face == bFace:
			continue
		for edge in face.edges:
			if bFace in edge.link_faces:
				connectedFaces.append(face)
	return connectedFaces

# returns false if not enough faces to check for creasing
def getNextFace(faceWeight, bVert, visitedFaces, availableFaces):
	connectedFaces = getConnectedFaces(faceWeight.face, bVert)
	visitedFaces.append(faceWeight.face)

	newFaceFound = False
	nextPrevFace = faceWeight.face
	for face in connectedFaces:
		if face in visitedFaces:
			continue
		elif not newFaceFound:
			newFaceFound = True
			faceWeight.prevFace = faceWeight.face
			faceWeight.face = face
		else:
			availableFaces.append(FaceWeight(face, nextPrevFace,
				faceWeight.weight))
	
	if not newFaceFound:
		availableFaces.remove(faceWeight)
	
	removedFaceWeights = []
	for otherFaceWeight in availableFaces:
		if otherFaceWeight.face in visitedFaces:
			removedFaceWeights.append(otherFaceWeight)
	for removedFaceWeight in removedFaceWeights:
		availableFaces.remove(removedFaceWeight)

def getLoopFromFaceVert(bFace, bVert):
	for loop in bFace.loops:
		if loop.vert == bVert:
			return loop
	return None

def getEdgeBetweenFaces(faceWeight):
	face1 = faceWeight.face
	face2 = faceWeight.prevFace
	for edge1 in face1.edges:
		for edge2 in face2.edges:
			if edge1 == edge2:
				return edge1
	return None

class FaceWeight:
	def __init__(self, face, prevFace, weight):
		self.face = face
		self.prevFace = prevFace
		self.weight = weight

def getWeightedNormalAndMoveToNextFace(selectFaceWeight, visitedFaces, 
	availableFaces, centerVert, edges):
	selectLoop = getLoopFromFaceVert(selectFaceWeight.face, centerVert)
	edgeIndex = getEdgeBetweenFaces(selectFaceWeight).index

	# Ignore triangulated faces
	if edgeIndex < len(edges):
		selectFaceWeight.weight *= 1 - edges[edgeIndex].crease

	getNextFace(selectFaceWeight, centerVert, visitedFaces, availableFaces)
	return selectLoop.calc_normal() * selectLoop.calc_angle() * \
		selectFaceWeight.weight

def getHighestFaceWeight(faceWeights):
	highestFaceWeight = faceWeights[0]
	for faceWeight in faceWeights[1:]:
		if faceWeight.weight > highestFaceWeight.weight:
			highestFaceWeight = faceWeight
	return highestFaceWeight
'''

def convertVertexData(mesh, loopPos, loopUV, loopColorOrNormal, 
	texDimensions, transformMatrix, isPointSampled, exportVertexColors):
	#uv_layer = mesh.uv_layers.active
	#color_layer = mesh.vertex_colors['Col']
	#alpha_layer = mesh.vertex_colors['Alpha']

	# Position (8 bytes)
	position = [int(round(floatValue)) for \
		floatValue in (transformMatrix @ loopPos)]

	# UV (4 bytes)
	# For F3D, Bilinear samples the point from the center of the pixel.
	# However, Point samples from the corner.
	# Thus we add 0.5 to the UV only if bilinear filtering.
	# see section 13.7.5.3 in programming manual.
	pixelOffset = 0 if isPointSampled else 0.5
	uv = [
		convertFloatToFixed16(loopUV[0] * texDimensions[0] - pixelOffset),
		convertFloatToFixed16(loopUV[1] * texDimensions[1] - pixelOffset)
	]

	# Color/Normal (4 bytes)
	if exportVertexColors:
		colorOrNormal = [
			int(round(loopColorOrNormal[0] * 255)).to_bytes(1, 'big')[0],
			int(round(loopColorOrNormal[1] * 255)).to_bytes(1, 'big')[0],
			int(round(loopColorOrNormal[2] * 255)).to_bytes(1, 'big')[0],
			int(round(loopColorOrNormal[3] * 255)).to_bytes(1, 'big')[0]
		]
	else:
		# normal transformed correctly.
		normal = (transformMatrix.inverted().transposed() @ \
			loopColorOrNormal).normalized()
		colorOrNormal = [
			int(round(normal[0] * 127)).to_bytes(1, 'big', signed = True)[0],
			int(round(normal[1] * 127)).to_bytes(1, 'big', signed = True)[0],
			int(round(normal[2] * 127)).to_bytes(1, 'big', signed = True)[0],
			0xFF
		]

	return Vtx(position, uv, colorOrNormal)

def getLoopColor(loop, mesh):
	color_layer = mesh.vertex_colors['Col'].data if 'Col' in \
		mesh.vertex_colors else None
	alpha_layer = mesh.vertex_colors['Alpha'].data if 'Alpha' in \
		mesh.vertex_colors else None

	if color_layer is not None:
		normalizedRGB = color_layer[loop.index].color
	else:
		normalizedRGB = [1,1,1]
	if alpha_layer is not None:
		normalizedAColor = alpha_layer[loop.index].color
		normalizedA = mathutils.Color(normalizedAColor[0:3]).v
	else:
		normalizedA = 1
	
	return (normalizedRGB[0], normalizedRGB[1], normalizedRGB[2], normalizedA)

def getLoopColorOrNormal(loop, face, mesh, obj, exportVertexColors):
	material = obj.data.materials[face.material_index]
	isFlatShaded = checkIfFlatShaded(material)
	if exportVertexColors:
		return getLoopColor(loop, mesh)
	else:
		return getLoopNormal(loop, face, isFlatShaded)

def createTriangleCommands(triangles, useSP2Triangle):
	triangles = copy.deepcopy(triangles)
	commands = []
	if useSP2Triangle:
		while len(triangles) > 0:
			if len(triangles) >= 2:
				commands.append(SP2Triangles(
					triangles[0][0], triangles[0][1], triangles[0][2], 0,
					triangles[1][0], triangles[1][1], triangles[1][2], 0))
				triangles = triangles[2:]
			else:
				commands.append(SP1Triangle(
					triangles[0][0], triangles[0][1], triangles[0][2], 0))
				triangles = []
	else:
		while len(triangles) > 0:
			commands.append(SP1Triangle(
				triangles[0][0], triangles[0][1], triangles[0][2], 0))
			triangles = triangles[1:]
		
	return commands

'''
def saveGeometry(obj, triList, vtxList, bFaces, bMesh, 
	texDimensions, transformMatrix, isPointSampled, isFlatShaded,
	exportVertexColors, f3d):

	uv_layer = bMesh.loops.layers.uv.verify()
	convertInfo = LoopConvertInfo(uv_layer, bMesh, obj, exportVertexColors)
	triangleConverter = TriangleConverter(obj.data, convertInfo, triList, 
		vtxList, f3d, texDimensions, transformMatrix, isPointSampled,
		exportVertexColors, None, None)

	for bFace in bFaces:
		triangleConverter.addFace(bFace)	
	triangleConverter.finish()
'''

# white diffuse, grey ambient, normal = (1,1,1)
defaultLighting = [
	(mathutils.Vector((1,1,1)), mathutils.Vector((1, 1, 1)).normalized()),
	(mathutils.Vector((0.5, 0.5, 0.5)), mathutils.Vector((1, 1, 1)).normalized())]

def saveOrGetF3DMaterial(material, fModel, obj):
	if material in fModel.materials:
		return fModel.materials[material]
	
	if len(obj.data.materials) == 0:
		raise ValueError("Mesh must have at least one material.")
	
	fMaterial = FMaterial(toAlnum(material.name))
	fMaterial.material.commands.append(DPPipeSync())
	fMaterial.revert.commands.append(DPPipeSync())
	
	if not material.is_f3d:
		raise ValueError("Material named " +  material.name + \
			' is not an F3D material.')
	nodes = material.node_tree.nodes

	if material.set_combiner:
		if material.rdp_settings.g_mdsft_cycletype == 'G_CYC_2CYCLE':
			fMaterial.material.commands.append(
				DPSetCombineMode(
					nodes['Case A 1'].inA,
					nodes['Case B 1'].inB,
					nodes['Case C 1'].inC,
					nodes['Case D 1'].inD,
					nodes['Case A Alpha 1'].inA_alpha,
					nodes['Case B Alpha 1'].inB_alpha,
					nodes['Case C Alpha 1'].inC_alpha,
					nodes['Case D Alpha 1'].inD_alpha,
					nodes['Case A 2'].inA,
					nodes['Case B 2'].inB,
					nodes['Case C 2'].inC,
					nodes['Case D 2'].inD,
					nodes['Case A Alpha 2'].inA_alpha,
					nodes['Case B Alpha 2'].inB_alpha,
					nodes['Case C Alpha 2'].inC_alpha,
					nodes['Case D Alpha 2'].inD_alpha
			))
		else:
			fMaterial.material.commands.append(
				DPSetCombineMode(
					nodes['Case A 1'].inA,
					nodes['Case B 1'].inB,
					nodes['Case C 1'].inC,
					nodes['Case D 1'].inD,
					nodes['Case A Alpha 1'].inA_alpha,
					nodes['Case B Alpha 1'].inB_alpha,
					nodes['Case C Alpha 1'].inC_alpha,
					nodes['Case D Alpha 1'].inD_alpha,
					nodes['Case A 1'].inA,
					nodes['Case B 1'].inB,
					nodes['Case C 1'].inC,
					nodes['Case D 1'].inD,
					nodes['Case A Alpha 1'].inA_alpha,
					nodes['Case B Alpha 1'].inB_alpha,
					nodes['Case C Alpha 1'].inC_alpha,
					nodes['Case D Alpha 1'].inD_alpha
			))

	if material.set_fog:
		fMaterial.material.commands.extend([
			DPSetFogColor(
				int(round(material.fog_color[0] * 255)),
				int(round(material.fog_color[1] * 255)),
				int(round(material.fog_color[2] * 255)),
				int(round(material.fog_color[3] * 255))),
			SPFogPosition(material.fog_position[0], material.fog_position[1])
		])

	useDict = all_combiner_uses(material,
		material.rdp_settings.g_mdsft_cycletype == '1')

	defaults = bpy.context.scene.world.rdp_defaults
	saveGeoModeDefinition(fMaterial, material.rdp_settings, defaults)
	saveOtherModeHDefinition(fMaterial, material.rdp_settings, defaults)
	saveOtherModeLDefinition(fMaterial, material.rdp_settings, defaults)

	# Set scale
	s = int(material.tex_scale[0] * 0xFFFF)
	t = int(material.tex_scale[1] * 0xFFFF)
	fMaterial.material.commands.append(
		SPTexture(s, t, 0, fModel.f3d.G_TX_RENDERTILE, 1))

	# Save textures
	texDimensions0, nextTmem = saveTextureIndex(useDict, material, fModel, 
		fMaterial, material.tex0, 0, 'Texture 0', 0)
	texDimensions1, nextTmem = saveTextureIndex(useDict, material, fModel, 
		fMaterial, material.tex1, 1, 'Texture 1', nextTmem)

	# Used so we know how to convert normalized UVs when saving verts.
	if texDimensions0 is not None and texDimensions1 is not None:
		texDimensions = texDimensions0 if material.uv_basis == 'TEXEL0' \
			else texDimensions1
	elif texDimensions0 is not None:
		texDimensions = texDimensions0
	elif texDimensions1 is not None:
		texDimensions = texDimensions1
	else:
		texDimensions = [32, 32]

	if useDict['Primitive'] and material.set_prim:
		color = nodes['Primitive Color'].outputs[0].default_value
		color = gammaCorrect(color[0:3]) + [color[3]]
		fMaterial.material.commands.append(
			DPSetPrimColor(
			int(material.prim_lod_min * 255),
			int(material.prim_lod_frac * 255),
			int(color[0] * 255), 
			int(color[1] * 255), 
			int(color[2] * 255),
			int(color[3] * 255)))

	if useDict['Environment'] and material.set_env:	
		color = nodes['Environment Color'].outputs[0].default_value
		color = gammaCorrect(color[0:3]) + [color[3]]
		fMaterial.material.commands.append(
			DPSetEnvColor(
			int(color[0] * 255), 
			int(color[1] * 255), 
			int(color[2] * 255),
			int(color[3] * 255)))
	
	if useDict['Shade'] and material.set_lights:
		fLights = saveLightsDefinition(fModel, material, 
			fMaterial.material.name + '_lights')
		fMaterial.material.commands.extend([
			SPSetLights(fLights) # TODO: handle synching: NO NEED?
		])
	
	if useDict['Key'] and material.set_key:
		center = nodes['Chroma Key Center'].outputs[0].default_value
		scale = nodes['Chroma Key Scale'].outputs[0].default_value
		width = material.key_width
		fMaterial.material.commands.extend([
			DPSetCombineKey('G_CK_KEY'),
			# TODO: Add UI handling width
			DPSetKeyR(int(center[0] * 255), int(scale[0] * 255), 
				int(width[0] * 2**8)),
			DPSetKeyGB(int(center[1] * 255), int(scale[1] * 255), 
				int(width[1] * 2**8), 
				int(center[2] * 255), int(scale[2] * 255), 
				int(width[2] * 2**8))	
		])

	# all k0-5 set at once
	# make sure to handle this in node shader
	# or don't, who cares
	if useDict['Convert'] and material.set_k0_5:
		fMaterial.material.commands.extend([
			DPSetTextureConvert('G_TC_FILTCONV'), # TODO: allow filter option
			DPSetConvert(
				int(material.k0 * 255),
				int(material.k1 * 255),
				int(material.k2 * 255),
				int(material.k3 * 255),
				int(material.k4 * 255),
				int(material.k5 * 255))
		])
		
	# End Display List
	fMaterial.material.commands.append(SPEndDisplayList())
	#revertMatAndEndDraw(fMaterial.revert)
	if len(fMaterial.revert.commands) > 1: # 1 being the pipe sync
		fMaterial.revert.commands.append(SPEndDisplayList())
	else:
		fMaterial.revert = None
	fModel.materials[material] = (fMaterial, texDimensions)

	return fMaterial, texDimensions

def saveTextureIndex(useDict, material, fModel, fMaterial, texProp, 
	index, name, tmem):
	if useDict[name] and texProp.tex_set:
		tex = texProp.tex
		if tex is None:
			raise ValueError('No ' + name + ' selected.')
		
		texFormat = texProp.tex_format
		isCITexture = texFormat[:2] == 'CI'
		palFormat = texProp.ci_format if isCITexture else ''
		texName = getNameFromPath(tex.filepath, True) + '_' + texFormat.lower()

		nextTmem = tmem + ceil(bitSizeDict[texBitSizeOf[texFormat]] * \
			tex.size[0] * tex.size[1] / 64) 
		if nextTmem > (512 if texFormat[:2] != 'CI' else 256):
			print(nextTmem)
			raise ValueError("Textures are too big. Max TMEM size is 4k " + \
				"bytes, ex. 2 32x32 RGBA 16 bit textures.")
		if tex.size[0] > 1024 or tex.size[1] > 1024:
			raise ValueError("Any side of an image cannot be greater " +\
				"than 1024.")

		clamp_S = texProp.S.clamp
		mirror_S = texProp.S.mirror
		tex_SL = texProp.S.low
		tex_SH = texProp.S.high
		mask_S = texProp.S.mask
		shift_S = texProp.S.shift

		clamp_T = texProp.T.clamp
		mirror_T = texProp.T.mirror
		tex_TL = texProp.T.low
		tex_TH = texProp.T.high
		mask_T = texProp.T.mask
		shift_T = texProp.T.shift

		if isCITexture:
			fImage, fPalette = saveOrGetPaletteDefinition(
				fModel, tex, texName, texFormat, palFormat)
			savePaletteLoading(fModel, fMaterial, fPalette, 
				palFormat, 0, fPalette.height, fModel.f3d)
		else:
			fImage = saveOrGetTextureDefinition(fModel, tex, texName, 
				texFormat)
		saveTextureLoading(fImage, fMaterial, clamp_S,
		 	mirror_S, clamp_T, mirror_T,
			mask_S, mask_T, shift_S, 
			shift_T, tex_SL, tex_TL, tex_SH, 
			tex_TH, texFormat, index, fModel.f3d, tmem)
		texDimensions = fImage.width, fImage.height
		#fImage = saveTextureDefinition(fModel, tex, texName, 
		#	texFormatOf[texFormat], texBitSizeOf[texFormat])
		#fModel.textures[texName] = fImage	
		
	else:
		texDimensions = None
		nextTmem = tmem

	return texDimensions, nextTmem

# texIndex: 0 for texture0, 1 for texture1
def saveTextureLoading(fImage, fMaterial, clamp_S, mirror_S, clamp_T,
	mirror_T, mask_S, mask_T, shift_S, shift_T,
	SL, TL, SH, TH, tex_format, texIndex, f3d, tmem):
	cms = [('G_TX_CLAMP' if clamp_S else 'G_TX_WRAP'),
		('G_TX_MIRROR' if mirror_S else 'G_TX_NOMIRROR')]
	cmt = [('G_TX_CLAMP' if clamp_T else 'G_TX_WRAP'),
		('G_TX_MIRROR' if mirror_T else 'G_TX_NOMIRROR')]
	masks = mask_S
	maskt = mask_T
	shifts = shift_S if shift_S > 0 else (shift_S + 16)
	shiftt = shift_T if shift_T > 0 else (shift_T + 16)

	#print('Low ' + str(SL) + ' ' + str(TL))
	sl = int(SL * (2 ** f3d.G_TEXTURE_IMAGE_FRAC))
	tl = int(TL * (2 ** f3d.G_TEXTURE_IMAGE_FRAC))
	sh = int(SH * (2 ** f3d.G_TEXTURE_IMAGE_FRAC))
	th = int(TH * (2 ** f3d.G_TEXTURE_IMAGE_FRAC))
	
	fmt = texFormatOf[tex_format]
	siz = texBitSizeOf[tex_format]
	pal = 0 if fmt[:2] != 'CI' else 0 # handle palettes
	dxt = f3d.CALC_DXT_4b(fImage.width) if siz == 'G_IM_SIZ_4b' else \
		f3d.CALC_DXT(fImage.width, f3d.G_IM_SIZ_VARS[siz + '_BYTES'])
	
	# Note that _LINE_BYTES and _TILE_BYTES variables are the same.
	line = (((fImage.width >> 1) + 7) >> 3) if siz == 'G_IM_SIZ_4b' else \
		(((fImage.width * f3d.G_IM_SIZ_VARS[siz + '_LINE_BYTES']) + 7) >> 3)

	# LoadTile will pad rows to 64 bit word alignment, while
	# LoadBlock assumes this is already done.

	fMaterial.material.commands.extend([
		DPTileSync(), # added in
		
		# Load Block version
		#DPSetTextureImage(fmt, siz + '_LOAD_BLOCK', 1, fImage),
		#DPSetTile(fmt, siz + '_LOAD_BLOCK', 0, tmem, 
		#	f3d.G_TX_LOADTILE - texIndex, 0, cmt, maskt, shiftt, 
		# 	cms, masks, shifts),
		#DPLoadSync(),
		#DPLoadBlock(f3d.G_TX_LOADTILE - texIndex, 0, 0, \
		#	(((fImage.width)*(fImage.height) + \
		#	f3d.G_IM_SIZ_VARS[siz + '_INCR']) >> \
		#	f3d.G_IM_SIZ_VARS[siz + '_SHIFT'])-1, \
		#	dxt),
		
		# Load Tile version
		DPSetTextureImage(fmt, siz, fImage.width, fImage),
		DPSetTile(fmt, siz + '_LOAD_BLOCK', line, tmem, 
			f3d.G_TX_LOADTILE - texIndex, 0, cmt, maskt, shiftt, 
		 	cms, masks, shifts),
		DPLoadSync(),
		DPLoadTile(f3d.G_TX_LOADTILE - texIndex, 0, 0,
			(fImage.width - 1) << f3d.G_TEXTURE_IMAGE_FRAC,
			(fImage.height - 1) << f3d.G_TEXTURE_IMAGE_FRAC),
			
		DPPipeSync(),
		DPSetTile(fmt, siz, line, tmem,	\
			f3d.G_TX_RENDERTILE + texIndex, pal, cmt, maskt, \
			shiftt, cms, masks, shifts),
		DPSetTileSize(f3d.G_TX_RENDERTILE + texIndex, sl, tl, sh, th)
		]) # added in

# palette stored in upper half of TMEM (words 256-511)
# pal is palette number (0-16), for CI8 always set to 0
def savePaletteLoading(fModel, fMaterial, fPalette, palFormat, pal, 
	colorCount, f3d):
	palFmt = texFormatOf[palFormat]
	cms = ['G_TX_WRAP', 'G_TX_NOMIRROR']
	cmt = ['G_TX_WRAP', 'G_TX_NOMIRROR']

	fMaterial.material.commands.append(DPSetTextureLUT(
		'G_TT_RGBA16' if palFmt == 'G_IM_FMT_RGBA' else 'G_TT_IA16'))
	fMaterial.revert.commands.append(DPSetTextureLUT('G_TT_NONE'))

	if not f3d._HW_VERSION_1:
		fMaterial.material.commands.extend([
			DPSetTextureImage(palFmt, 'G_IM_SIZ_16b', 1, fPalette),
			DPTileSync(),
			DPSetTile('0', '0', 0, (256+(((pal)&0xf)*16)),\
				f3d.G_TX_LOADTILE, 0, cmt, 0, 0, cms, 0, 0),
			DPLoadSync(),
			DPLoadTLUTCmd(f3d.G_TX_LOADTILE, colorCount - 1),
			DPPipeSync()])
	else:
		fMaterial.material.commands.extend([
			_DPLoadTextureBlock(fPalette, \
				(256+(((pal)&0xf)*16)), \
            	palFmt, 'G_IM_SIZ_16b', 4*colorCount, 1,
            	pal, cms, cmt, 0, 0, 0, 0)])
	
def saveOrGetPaletteDefinition(fModel, image, imageName, texFmt, palFmt):
	texFormat = texFormatOf[texFmt]
	palFormat = texFormatOf[palFmt]
	bitSize = texBitSizeOf[texFmt]
	# If image already loaded, return that data.
	paletteName = toAlnum(imageName) + '_pal_' + palFmt.lower()
	imageKey = (image, (texFmt, palFmt))
	palKey = (image, (palFmt, 'PAL'))
	if imageKey in fModel.textures:
		return fModel.textures[imageKey], fModel.textures[palKey]

	palette = []
	texture = []
	maxColors = 16 if bitSize == 'G_IM_SIZ_4b' else 256
	for i in range(image.size[0] * image.size[1]):
		color = [1,1,1,1]
		for field in range(image.channels):
			color[field] = image.pixels[i * image.channels + field]
		if palFormat == 'G_IM_FMT_RGBA':
			pixelColor = \
				((int(color[0] * 0x1F) & 0x1F) << 11) | \
				((int(color[1] * 0x1F) & 0x1F) << 6) | \
				((int(color[2] * 0x1F) & 0x1F) << 1) | \
				(1 if color[3] > 0.5 else 0)
		elif palFormat == 'G_IM_FMT_IA':
			intensity = mathutils.Color(color[0:3]).v
			alpha = color[3]
			pixelColor = (int(intensity * 0xFF) << 8) | int(alpha * 0xFF)
		else:
			raise ValueError("Invalid combo: " + palFormat + ', ' + bitSize)

		if pixelColor not in palette:
			palette.append(pixelColor)
			if len(palette) > maxColors:
				raise ValueError('Texture ' + imageName + ' has more than ' + \
					str(maxColors) + ' colors.')
		texture.append(palette.index(pixelColor))
	
	filename = getNameFromPath(image.filepath, True) + '.' + \
		texFmt.lower() + '.inc.c'
	paletteFilename = getNameFromPath(image.filepath, True) + '.' + \
		texFmt.lower() + '.pal'
	fImage = FImage(toAlnum(imageName), texFormat, bitSize, 
		image.size[0], image.size[1], filename)

	fPalette = FImage(paletteName, palFormat, 'G_IM_SIZ_16b', 1, 
		len(palette), paletteFilename)
	#paletteTex = bpy.data.images.new(paletteName, 1, len(palette))
	#paletteTex.pixels = palette
	#paletteTex.filepath = getNameFromPath(image.filepath, True) + '.' + \
	#	texFmt.lower() + '.pal'

	for color in palette:
		fPalette.data.extend(color.to_bytes(2, 'big')) 
	
	if bitSize == 'G_IM_SIZ_4b':
		fImage.data = compactNibbleArray(texture, image.size[0], image.size[1])
	else:	
		fImage.data = bytearray(texture)
	
	fModel.textures[(image, (texFmt, palFmt))] = fImage
	fModel.textures[(image, (palFmt, 'PAL'))] = fPalette

	return fImage, fPalette #, paletteTex

def compactNibbleArray(texture, width, height):
	nibbleData = bytearray(0)
	dataSize = int(width * height / 2)

	for i in range(dataSize):
		nibbleData.append(
			((texture[i * 2] & 0xF) << 4) |\
			(texture[i * 2 + 1] & 0xF)
		)

	if (width * height) % 2 == 1:
		nibbleData.append((texture[-1] & 0xF) << 4)
	
	return nibbleData

def saveOrGetTextureDefinition(fModel, image, imageName, texFormat):
	fmt = texFormatOf[texFormat]
	bitSize = texBitSizeOf[texFormat]

	# If image already loaded, return that data.
	imageKey = (image, (texFormat, 'NONE'))
	if imageKey in fModel.textures:
		return fModel.textures[imageKey]

	filename = getNameFromPath(image.filepath, True) + '.' + \
		texFormat.lower() + '.inc.c'

	fImage = FImage(toAlnum(imageName), fmt, bitSize, 
		image.size[0], image.size[1], filename)

	for i in range(image.size[0] * image.size[1]):
		color = [1,1,1,1]
		for field in range(image.channels):
			color[field] = image.pixels[i * image.channels + field]
		if fmt == 'G_IM_FMT_RGBA':
			if bitSize == 'G_IM_SIZ_16b':
				words = \
					((int(color[0] * 0x1F) & 0x1F) << 11) | \
					((int(color[1] * 0x1F) & 0x1F) << 6) | \
					((int(color[2] * 0x1F) & 0x1F) << 1) | \
					(1 if color[3] > 0.5 else 0)
				fImage.data.extend(bytearray(words.to_bytes(2, 'big')))
			elif bitSize == 'G_IM_SIZ_32b':
				fImage.data.extend(bytearray([
					int(value * 0xFF) & 0xFF for value in color]))
			else:
				raise ValueError("Invalid combo: " + fmt + ', ' + bitSize)

		elif fmt == 'G_IM_FMT_YUV':
			raise ValueError("YUV not yet implemented.")
			if bitSize == 'G_IM_SIZ_16b':
				pass
			else:
				raise ValueError("Invalid combo: " + fmt + ', ' + bitSize)

		elif fmt == 'G_IM_FMT_CI':
			raise ValueError("CI not yet implemented.")

		elif fmt == 'G_IM_FMT_IA':
			intensity = mathutils.Color(color[0:3]).v
			alpha = color[3]
			if bitSize == 'G_IM_SIZ_4b':
				fImage.data.append(
					((int(intensity * 0x7) & 0x7) << 1) | \
					(1 if alpha > 0.5 else 0))
			elif bitSize == 'G_IM_SIZ_8b':
				fImage.data.append(
					((int(intensity * 0xF) & 0xF) << 4) | \
					(int(alpha * 0xF) & 0xF))
			elif bitSize == 'G_IM_SIZ_16b':
				fImage.data.extend(bytearray(
					[int(intensity * 0xFF), int(alpha * 0xFF)]))
			else:
				raise ValueError("Invalid combo: " + fmt + ', ' + bitSize)
		elif fmt == 'G_IM_FMT_I':
			intensity = mathutils.Color(color[0:3]).v
			if bitSize == 'G_IM_SIZ_4b':
				fImage.data.append(int(intensity * 0xF))
			elif bitSize == 'G_IM_SIZ_8b':
				fImage.data.append(int(intensity * 0xFF))
			else:
				raise ValueError("Invalid combo: " + fmt + ', ' + bitSize)
		else:
			raise ValueError("Invalid image format " + fmt)
	
	# We stored 4bit values in byte arrays, now to convert
	if bitSize == 'G_IM_SIZ_4b':
		fImage.data = \
			compactNibbleArray(fImage.data, image.size[0], image.size[1])
	
	fModel.textures[(image, (texFormat, 'NONE'))] = fImage
	return fImage

def saveLightsDefinition(fModel, material, lightsName):
	nodes = material.node_tree.nodes
	ambientColor = gammaCorrect(nodes['Ambient Color'].outputs[0].default_value)
	lights = Lights(toAlnum(lightsName))
	lights.a = Ambient(
		[int(ambientColor[0] * 255),
		int(ambientColor[1] * 255),
		int(ambientColor[2] * 255)])
	
	if material.f3d_light1 is not None:
		addLightDefinition(material.f3d_light1, lights)
	if material.f3d_light2 is not None:
		addLightDefinition(material.f3d_light2, lights)
	if material.f3d_light3 is not None:
		addLightDefinition(material.f3d_light3, lights)
	if material.f3d_light4 is not None:
		addLightDefinition(material.f3d_light4, lights)
	if material.f3d_light5 is not None:
		addLightDefinition(material.f3d_light5, lights)
	if material.f3d_light6 is not None:
		addLightDefinition(material.f3d_light6, lights)
	if material.f3d_light7 is not None:
		addLightDefinition(material.f3d_light7, lights)
	
	fModel.lights[lightsName] = lights
	return lights

def addLightDefinition(f3d_light, fLights):
	lightObj = None
	for obj in bpy.context.scene.objects:
		if obj.data == f3d_light:
			lightObj = obj
			break
	if lightObj is None:
		raise ValueError(
			"You are referencing a light that is no longer in the scene.")
	
	#spaceRot = blenderToSM64Rotation.to_4x4().to_quaternion()
	spaceRot = mathutils.Euler((-pi / 2, 0, 0)).to_quaternion()
	rotation = spaceRot @ getObjectQuaternion(lightObj)
		
	normal = (rotation @ mathutils.Vector((0,0,-1))).normalized()
	color = gammaCorrect(f3d_light.color)
	
	fLights.l.append(Light(
		[
			int(color[0] * 255),
			int(color[1] * 255),
			int(color[2] * 255)
		],
		[
			# Make sure to handle negative values
			int.from_bytes(int(normal[0] * 127).to_bytes(1, 'big', 
				signed = True), 'big'),
			int.from_bytes(int(normal[1] * 127).to_bytes(1, 'big', 
				signed = True), 'big'),
			int.from_bytes(int(normal[2] * 127).to_bytes(1, 'big', 
				signed = True), 'big')
		],
	))

def saveBitGeo(value, defaultValue, flagName, setGeo, clearGeo):
	if value != defaultValue:
		if value:
			setGeo.flagList.append(flagName)
		else:
			clearGeo.flagList.append(flagName)

def saveGeoModeDefinition(fMaterial, settings, defaults):
	setGeo = SPSetGeometryMode([])
	clearGeo = SPClearGeometryMode([])

	saveBitGeo(settings.g_zbuffer, defaults.g_zbuffer, 'G_ZBUFFER',
		setGeo, clearGeo)
	saveBitGeo(settings.g_shade, defaults.g_shade, 'G_SHADE',
		setGeo, clearGeo)
	saveBitGeo(settings.g_cull_front, defaults.g_cull_front, 'G_CULL_FRONT',
		setGeo, clearGeo)
	saveBitGeo(settings.g_cull_back,  defaults.g_cull_back, 'G_CULL_BACK',
		setGeo, clearGeo)
	saveBitGeo(settings.g_fog, defaults.g_fog, 'G_FOG', setGeo, clearGeo)
	saveBitGeo(settings.g_lighting, defaults.g_lighting, 'G_LIGHTING',
		setGeo, clearGeo)

	# make sure normals are saved correctly.
	saveBitGeo(settings.g_tex_gen, defaults.g_tex_gen, 'G_TEXTURE_GEN', 
		setGeo, clearGeo)
	saveBitGeo(settings.g_tex_gen_linear, defaults.g_tex_gen_linear,
		'G_TEXTURE_GEN_LINEAR', setGeo, clearGeo)
	saveBitGeo(settings.g_shade_smooth, defaults.g_shade_smooth,
		'G_SHADING_SMOOTH', setGeo, clearGeo)
	if bpy.context.scene.f3d_type == 'F3DEX_GBI_2' or \
		bpy.context.scene.f3d_type == 'F3DEX_GBI':
		saveBitGeo(settings.g_clipping, defaults.g_clipping, 'G_CLIPPING', 
			setGeo, clearGeo)

	if len(setGeo.flagList) > 0:
		fMaterial.material.commands.append(setGeo)
		fMaterial.revert.commands.append(SPClearGeometryMode(setGeo.flagList))
	if len(clearGeo.flagList) > 0:
		fMaterial.material.commands.append(clearGeo)
		fMaterial.revert.commands.append(SPSetGeometryMode(clearGeo.flagList))

def saveModeSetting(fMaterial, value, defaultValue, cmdClass):
	if value != defaultValue:
		fMaterial.material.commands.append(cmdClass(value))
		fMaterial.revert.commands.append(cmdClass(defaultValue))

def saveOtherModeHDefinition(fMaterial, settings, defaults):
	saveModeSetting(fMaterial, settings.g_mdsft_alpha_dither,
		defaults.g_mdsft_alpha_dither, DPSetAlphaDither)

	if not bpy.context.scene.isHWv1:
		saveModeSetting(fMaterial, settings.g_mdsft_rgb_dither,
			defaults.g_mdsft_rgb_dither, DPSetColorDither)

		saveModeSetting(fMaterial, settings.g_mdsft_combkey,
			defaults.g_mdsft_combkey, DPSetCombineKey)
	
	saveModeSetting(fMaterial, settings.g_mdsft_textconv,
		defaults.g_mdsft_textconv, DPSetTextureConvert)

	saveModeSetting(fMaterial, settings.g_mdsft_text_filt,
		defaults.g_mdsft_text_filt, DPSetTextureFilter)

	#saveModeSetting(fMaterial, settings.g_mdsft_textlut,
	#	defaults.g_mdsft_textlut, DPSetTextureLUT)

	saveModeSetting(fMaterial, settings.g_mdsft_textlod,
		defaults.g_mdsft_textlod, DPSetTextureLOD)

	saveModeSetting(fMaterial, settings.g_mdsft_textdetail,
		defaults.g_mdsft_textdetail, DPSetTextureDetail)

	saveModeSetting(fMaterial, settings.g_mdsft_textpersp,
		defaults.g_mdsft_textpersp, DPSetTexturePersp)
	
	saveModeSetting(fMaterial, settings.g_mdsft_cycletype,
		defaults.g_mdsft_cycletype, DPSetCycleType)

	if bpy.context.scene.isHWv1:
		saveModeSetting(fMaterial, settings.g_mdsft_color_dither,
			defaults.g_mdsft_color_dither, DPSetColorDither)
	
	saveModeSetting(fMaterial, settings.g_mdsft_pipeline,
		defaults.g_mdsft_pipeline, DPPipelineMode)

def saveOtherModeLDefinition(fMaterial, settings, defaults):
	saveModeSetting(fMaterial, settings.g_mdsft_alpha_compare,
		defaults.g_mdsft_alpha_compare, DPSetAlphaCompare)

	saveModeSetting(fMaterial, settings.g_mdsft_zsrcsel,
		defaults.g_mdsft_zsrcsel, DPSetDepthSource)

	# cycle independent
	if settings.set_rendermode:
		if settings.g_mdsft_cycletype == 'G_CYC_2CYCLE':
			renderModeSet = DPSetRenderMode([], 
				settings.blend_p1, settings.blend_a1, 
				settings.blend_m1, settings.blend_b1,
				settings.blend_p2, settings.blend_a2, 
				settings.blend_m2, settings.blend_b2)
		else:
			renderModeSet = DPSetRenderMode([], 
				settings.blend_p1, settings.blend_a1, 
				settings.blend_m1, settings.blend_b1,
				settings.blend_p1, settings.blend_a1, 
				settings.blend_m1, settings.blend_b1)

		if settings.aa_en:
			renderModeSet.flagList.append("AA_EN")
		if settings.z_cmp:
			renderModeSet.flagList.append("Z_CMP")
		if settings.z_upd:
			renderModeSet.flagList.append("Z_UPD")
		if settings.im_rd:
			renderModeSet.flagList.append("IM_RD")
		if settings.clr_on_cvg:
			renderModeSet.flagList.append("CLR_ON_CVG")

		renderModeSet.flagList.append(settings.cvg_dst)
		renderModeSet.flagList.append(settings.zmode)
		
		if settings.cvg_x_alpha:
			renderModeSet.flagList.append("CVG_X_ALPHA")
		if settings.alpha_cvg_sel:
			renderModeSet.flagList.append("ALPHA_CVG_SEL")
		if settings.force_bl:
			renderModeSet.flagList.append("FORCE_BL")
		
		fMaterial.material.commands.append(renderModeSet)
		#fMaterial.revert.commands.append(defaultRenderMode)