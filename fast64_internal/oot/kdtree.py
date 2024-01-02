from __future__ import annotations
from .oot_collision_classes import OOTCollisionPolygon, OOTCollisionVertex
from typing import Optional, List, Tuple, Dict
import mathutils

MIN_POLYS_PER_LEAF = 5
MAX_DEPTH = 10

class KDNode_C:
    def __init__(
        self,
        axis: Optional[int] = None,
        splitValue: Optional[int] = None,
        leftIndex: Optional[int] = None,
        rightIndex: Optional[int] = None,
        polyIndices: Optional[str] = None,
        numPolys: Optional[int] = None
    ):
        if numPolys is None or numPolys == 0:
            self.axis = str(axis)
            self.splitValue = str(splitValue)
            self.leftIndex = str(leftIndex)
            self.rightIndex = str(rightIndex)
            self.polyIndices = "NULL"
            self.numPolys = "0"
        else:
            self.axis = "0xFFFF"
            self.splitValue = "0xFFFF"
            self.leftIndex = "0xFFFF"
            self.rightIndex = "0xFFFF"
            self.polyIndices = f"&{polyIndices}"
            self.numPolys = str(numPolys)

    def to_c_str(self):
        return "{" + f"{self.axis}, {self.splitValue}, {self.leftIndex}, {self.rightIndex}, {self.polyIndices}, {self.numPolys}" + "}"


class KDNode:
  def __init__(
    self,
    axis: Optional[int],
    split_value: Optional[int],
    left: Optional[KDNode],
    right: Optional[KDNode],
    polygons: Optional[List[Tuple[int, OOTCollisionPolygon]]]
  ):
    self.axis = axis
    self.split_value = split_value
    self.left = left
    self.right = right
    self.polygons = polygons


  @staticmethod
  def construct_tree(
    polygons: List[Tuple[int, OOTCollisionPolygon]],
    vertices: List[OOTCollisionVertex],
    axis: int = 0, depth: int = 0
  ):
    def sort_polys_key(polygon: OOTCollisionPolygon):
      nonlocal axis
      nonlocal vertices
      return max([
        vertices[polygon.indices[0]].position[axis],
        vertices[polygon.indices[1]].position[axis],
        vertices[polygon.indices[2]].position[axis],
      ])

    def extract_verts(poly: OOTCollisionPolygon) -> List[int]:
      return (vertices[i].position[axis] for i in poly.indices)

    # become an leaf node
    if len(polygons) <= MIN_POLYS_PER_LEAF or depth >= MAX_DEPTH:
      return KDNode(None, None, None, None, polygons)
    
    for retry in range(3):
      # try to split on a point that roughly balances the number of
      # tris in each partition, but add a little jitter (+1) so its
      # not exactly on an actual vert position to reduce dupes
      sorted_polys = sorted((it[1] for it in polygons), key=sort_polys_key)
      split_value = 1 + max(extract_verts(sorted_polys[len(sorted_polys) // 2]))
      
      lte, gte = [], []
      for poly in polygons:
        max_value = max(extract_verts(poly[1]))
        min_value = min(extract_verts(poly[1]))
        # put it in both partitions if exactly on a split or straddles one
        stored = False
        if max_value < split_value:
          stored = True
          lte.append(poly)
        if min_value > split_value:
          stored = True
          gte.append(poly)
        if not stored:
          lte.append(poly)
          gte.append(poly)
 
      # can't subdivide any further on this axis, try another
      if len(lte) == 0 or len(gte) == 0:
        return KDNode(None, None, None, None, polygons)
        axis = (axis + 1) % 3
      else:
          break
      
    # couldn't subdivide on any axis, become an leaf
    if retry == 2:
      return KDNode(None, None, None, None, polygons)

    next_axis = 2 if axis == 0 else 0
    left = None if len(lte) == 0 else KDNode.construct_tree(lte, vertices, next_axis, depth + 1)
    right = None if len(gte) == 0 else KDNode.construct_tree(gte, vertices, next_axis, depth + 1)
    
    return KDNode(axis, split_value, left, right, None)


  def show(self):
        axes = ["x", "y", "z"]
        num_polys_in_tree = 0
        def _show(node: KDNode, indent: str, side: str):
            nonlocal num_polys_in_tree

            if node is None:
                return

            if node.polygons is not None:
                num_polys_in_tree += len(node.polygons)
                for poly in node.polygons:
                    print(f"{indent}[{side}]- {poly[0]=}")
            else:
                print(f"{indent}[{side}]- Split at {axes[node.axis]} = {node.split_value}")
                _show(node.left, indent + "    ", "L")
                _show(node.right, indent + "    ", "R")

        print(f"=======")
        _show(self, "", None)
        print(f"======= Total polys: {num_polys_in_tree}")


  def to_c_parts(self, ordered_polygons: List[Tuple[int, OOTCollisionPolygon]], owner_name: str):
        poly_indices: Dict[str, List[int]] = {}
        nodes: List[KDNode_C] = []

        def _visit(node: KDNode, path: str = "") -> int:
            if node.polygons is not None:
                name = f"{owner_name}_polyIndexList_{path}"
                indices = [it[0] for it in node.polygons]
                poly_indices[name] = indices
                node_c = KDNode_C(polyIndices=name, numPolys=len(indices))
            else:
                left_id, right_id = None, None
                if node.left is not None:
                    left_id = _visit(node.left, path + "L")
                if node.right is not None:
                    right_id = _visit(node.right, path + "R")
                
                node_c = KDNode_C(axis=node.axis, splitValue=node.split_value, leftIndex=left_id, rightIndex=right_id)
 
            nodes.append(node_c)
            return len(nodes) - 1
 
        _visit(self)
        return poly_indices, f"{owner_name}_nodeList", nodes
