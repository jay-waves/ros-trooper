#!/usr/bin/python3
'''
3. 根据格式树解析输入的消息(比如yaml), 也支持将解析后的消息重新输出为yaml. 要求可以从命令行用管道命令输入
4. 对于读取的yaml格式消息, 支持存为种子(文件)
5. 支持变异种子, 比如该消息格式为 int32[], 支持将列表长度和数据全部变异(控制大小的随机数)

如: 
$ ros2 interface show std_msgs/Header
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

解析为:
MsgTree.name = std_msgs/Header
MsgTree.root = [{
    type: std_msgs/Header,
    name: header,
    submsgs: [{
        type: builtin_interfaces/Time,
        name: stamp,
        submsgs: [{
            type: int32,
            name: sec,
            submsgs: []
        },{
            type: uint32,
            name: nanosec,
            submsgs: []
        }]
    },{
        type: string,
        name: frame_id,
        submsgs: []
    }]
},]
'''

from __future__ import annotations
from typing import Iterator, Tuple
import yaml
import copy
import struct
import subprocess


class MsgNode:
	'''消息节点, 及其子节点'''
	types_map = {
			'uint8': 'B',  # unsigned char
			'int8': 'b',   # signed char
			'int16': 'h',  # short
			'uint16': 'H', # unsigned short
			'int32': 'i',
			'uint32': 'I',
			'int64': 'q',  # long long
			'uint64': 'Q', # unsigned long long
			'float64': 'd',
			'float32': 'f',
			'string': 's',
			'_Bool': '?',
	}

	def __init__(self, type, name, submsgs =None, data=None):
		self.type: str = type
		self.name: str = name
		self._data = None
		self.data = data # use data.setter, for default value
		self._submsgs = {} if submsgs is None else submsgs

	def __repr__(self):
		return f"MsgNode({self.type}, {self.name}, {self._data}, {self._submsgs})"

	def __iter__(self) -> Iterator[Tuple[str, MsgNode]]:
		return iter(self._submsgs.items())
	
	def copy_without_data(self):
		new_submsgs = {}
		for name, submsg in self._submsgs.items():
			if isinstance(submsg, MsgNode):
				new_submsgs[name] = submsg.copy_without_data()
			elif isinstance(submsg, list):
				new_submsgs[name] = [msg.copy_without_data() for msg in submsg]
			else:
				raise ValueError("Unrecognized type")
		new_copy = copy.copy(self)
		new_copy.data = None
		new_copy._submsgs = new_submsgs
		return new_copy
	
	def copy_with_data(self):
		return copy.deepcopy(self)

	def add_submsg(self, name, subnode):
		if name in self._submsgs:
			raise ValueError(f"submsg {name} already exists")
		self._submsgs[name]=subnode
	
	def get_submsg(self, name:str):
		if name not in self._submsgs:
			raise ValueError(f"submsg {name} not exists")
		return self._submsgs[name]

	@property
	def data(self):
		if self._data is None:
			return None
		return self._unpack_data(self.type, self._data)

	@data.setter
	def data(self, data):
		if data is None:
			self._data = None
		else:
			# make sure type not endswith [*]
			self._data = self._pack_data(type, data)
	
	@classmethod
	def _pack_data(cls, dtype, value):
		fmt_str = cls.types_map.get(dtype, None)
		if fmt_str is None:
			raise ValueError("dtype not existed")
		if dtype == 'string':
			bytes_value = value.encode('utf-8')
			return struct.pack(f"{len(value)}{fmt_str}", bytes_value)
		else:
			return struct.pack(fmt_str, value)

	@classmethod
	def _unpack_data(cls, dtype, packed_data):
		fmt_str = cls.types_map.get(dtype)
		if fmt_str is None:
			raise ValueError("dtype not existed")
		if dtype == 'string':
			str_len = struct.calcsize(fmt_str)
			return struct.unpack(f"{str_len}{fmt_str}", packed_data)[0]
		else:
			return struct.unpack(fmt_str, packed_data)[0]
	
	def _traverse_submsgs(self, fn):
		fn(self)
		for _, submsg in self:
			if isinstance(submsg, MsgNode):
				submsg._traverse_submsgs(fn)
			elif isinstance(submsg, list):
				for msg in submsg:
					msg._traverse_submsgs(fn)
			else:
				raise ValueError("Unrecognized type")
	
	def to_dict(self):
		submsgs = {}
		for name, submsg in self:
			if isinstance(submsg, MsgNode):
				submsgs[name] = submsg.to_msg()
			elif isinstance(submsg, list):
				submsgs[name] = [msg.to_msg() for msg in submsg]
			else:
				raise ValueError("Unrecognized type")
		return {
			key: value for key, value in [
				("type", self.type),
				("name", self.name),
				("data", self.data),
				("submsgs", submsgs),
			] if value is not None and value!=[] # remove empty element
		}
	
	def to_yaml(self):
		return yaml.dump(self.to_dict())	

	#TODO 写一个用来遍历解析submsgs的工具?
	def to_msg(self):
		'''msg in yaml format'''
		if self.data is None:
			submsgs = {}
			for name, submsg in self:
				if isinstance(submsg, MsgNode):
					submsgs[name] = submsg.to_msg()
				elif isinstance(submsg, list):
					submsgs[name] = [msg.to_msg() for msg in submsg]
				else:
					raise ValueError("Unrecognized type")
			return yaml.dump({self.name: submsgs})
		else:
			return yaml.dump({self.name: self.data})

	# @classmethod
	# def from_yaml(cls, yaml_str):
	# 	dict = yaml.safe_load(yaml_str)
	# 	return cls.from_dict(dict)


class MsgTree:
	'''维护一个完整消息的格式树, 提供遍历等整体功能'''
	def __init__(self, type: str, intrf=None):
		self.type = type
		# allow accept intrf outside
		self._submsgs = intrf if intrf else {}

	def __repr__(self):
		return f"MsgTree({self.type}, {self._submsgs})"
	
	def __iter__(self) -> Iterator[Tuple[str, MsgNode]]:
		return iter(self._submsgs.items())
	
	def add_submsg(self, name, topnode):
		if name in self._submsgs:
			raise ValueError(f"topnode {name} already exists")
		self._submsgs[name]=topnode
	
	def get_submsg(self, name:str):
		if name not in self._submsgs:
			raise ValueError(f"submsg {name} not exists")
		return self._submsgs[name]
	
	def get_intrf(self):
		new_intrf = {}
		for name, topmsg in self:
			if isinstance(topmsg, MsgNode):
				new_intrf[name] = topmsg.copy_without_data()
			elif isinstance(topmsg, list):
				new_intrf[name] = [submsg.copy_without_data() for submsg in topmsg]
			else:
				raise ValueError("Unrecognized type")
		return new_intrf
	
	# def get_node(self, path:list[str]):
	# 	'''get a node by its tree name path'''
	# 	# untest
	# 	def move_down(node, subpath):
	# 		if not subpath: # empty list
	# 			return None
	# 		for subnode in node:
	# 			if subnode.name == subpath[0]:
	# 				if len(subpath) == 1:
	# 					return subnode
	# 				else:
	# 					return move_down(subnode, subpath[1:])
	# 		return None
	# 	return move_down(self.root, path)

	# def mutate(self, mutator):
	# 	mutator(self._topnodes)
	
	# def to_yaml(self):
	# 	# only convert tree of msg_nodes
	# 	root_yaml = []
	# 	for root_node in self._topnodes:
	# 		root_yaml.append(root_node.to_yaml())
	# 	return yaml.dump(root_yaml)
	
	#TODO 捕捉异常(比如接口格式错误), 返回bool? 怎么检测格式错误
	def load_interface(self, interface_str: str):
		stack = []
		for line in interface_str.splitlines():
			# skip empty lines and comments
			if line.strip() == '' or line.strip().startswith('#'):
				continue
			indent_level = len(line) - len(line.lstrip())
			#unpack fields, notice defaultvalue field maybe []
			type, name, *dflt_data = line.strip().split() 
			dflt_data = dflt_data[0] if dflt_data else None
			node = MsgNode(type, name)
			# find parent
			if indent_level == 0:
				parent = self
			else: #nest field
				while stack and stack[-1][0] >= indent_level:
					stack.pop()
				parent = stack[-1][1] if stack else self
			# handle list
			if type.endswith(']'): 
				#! may have fixed size [], like uint8[8], or uint8[<=1]
				type = type.split('[', 1)[0]
				typenode = MsgNode(type.rstrip('[]'), name)
				node = [typenode]
				stack.append((indent_level, typenode)) # indent, parent
			else:
				node = MsgNode(type, name)
				stack.append((indent_level, node)) 
			parent.add_submsg(name, node)

	def load_yaml_data(self, yaml_str)->bool:
		root = yaml.safe_load(yaml_str)
		if root is None:
			return False
		return self.load_data(root, self)
	
	def load_data(self, data, typetree:MsgNode)->bool:
		'''ddcit: data dict, like from yaml
		mdict: msg node or msg tree's dict without data'''
		try:
			if isinstance(data, dict):
				for name, subdata in data.items():
					subtype = typetree.get_submsg(name)
					self.load_data(subdata, subtype)
			elif isinstance(data, list):
				if not isinstance(typetree, list):
					raise ValueError(f"submsg {name} isnot list, not incompatable")
				# 扩展现有格式树列表大小, 使其匹配数据大小
				subtype = typetree[0]
				typetree.clear()
				typetree.extend(
					[copy.copy(subtype) for _ in range(len(data))])
				for subdata, subtype in zip(data, typetree):
					self.load_data(subdata, subtype)
			else:
				# 总是更新数据
				typetree.data = data
		except ValueError as e:
			print(e)
			return False
		finally:
			return True



'''test'''

seed_name = "nav2_msgs/msg/Costmap"
seed = """
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id
CostmapMetaData metadata
        builtin_interfaces/Time map_load_time
                int32 sec
                uint32 nanosec
        builtin_interfaces/Time update_time
                int32 sec
                uint32 nanosec
        string layer
        float32 resolution
        uint32 size_x
        uint32 size_y
        geometry_msgs/Pose origin
                Point position
                        float64 x
                        float64 y
                        float64 z
                Quaternion orientation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1
uint8[] data
"""
em_str = ''
seed_data = """
header:
  stamp:
    sec: 16
    nanosec: 500000000
  frame_id: "map"
metadata:
  map_load_time:
    sec: 1638400000
    nanosec: 0
  update_time:
    sec: 1638400005
    nanosec: 500000000
  layer: "base_layer"
  resolution: 0.05
  size_x: 3
  size_y: 3
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data:
  - 0
  - 1
  - 2
  - 3
  - 4
  - 5
  - 6
  - 7
  - 8
"""

def test():
	msg = MsgTree(seed_name)
	msg.load_interface(seed)
	print(msg.load_yaml_data(seed_data))
	print(msg.load_yaml_data(em_str))
	print(msg)
	new_msg = msg.get_intrf()
	print(new_msg)
	print(msg)

test()