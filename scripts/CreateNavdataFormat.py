import re
from mako.template import Template

PATH_NAVDATA_KEYS = '../ARDroneLib/Soft/Common/navdata_keys.h'
PATH_NAVDATA_COMMON = '../ARDroneLib/Soft/Common/navdata_common.h'

keys = []

r_keys = re.compile(r'''
			^\s*[^#]?\s*?
			(NAVDATA_OPTION(?:_DEMO)?)
			\s*\(\s*
			((?P<structure>\w+))
			\s*,\s*
			((?P<name>\w+))
			\s*,\s*
			((?P<tag>\w+))
			\s*\)\s*$
			''', re.X)

print 'Parsing Navdata Struct Names'
with open(PATH_NAVDATA_KEYS,'r') as navdata_keys:
	for line in navdata_keys:
		matches = re.search(r_keys,line)
		if matches:
			print '-- '+matches.group('name')
			keys.append(matches.group('structure','name','tag'))

contents = ''
with open(PATH_NAVDATA_COMMON,'r') as navdata_common:
	contents = navdata_common.read()

ros_datatypes = {'uint32_t'		:('std_msgs::UInt32','uint32_t'),
				 'uint16_t'		:('std_msgs::UInt16','uint16_t'),
				 'uint8_t' 		:('std_msgs::UInt8','uint8_t'),
				 'int32_t'		:('std_msgs::Int32','int32_t'),
				 'int16_t'		:('std_msgs::Int16','int16_t'),
				 'int8_t' 		:('std_msgs::Int8','int8_t'),
				 'float64_t'	:('std_msgs::Float64','float64_t'),
				 'float32_t'	:('std_msgs::Float32','float32_t'),
				 'matrix33_t'	:('ardrone_autonomy::matrix33','ardrone_autonomy::matrix33'),
				 'vector31_t'	:('ardrone_autonomy::vector31','ardrone_autonomy::vector31'),
				 'vector21_t'	:('ardrone_autonomy::vector21','ardrone_autonomy::vector21'),
				 'screen_point_t'	:('ardrone_autonomy::vector21','ardrone_autonomy::vector21'),
				 'velocities_t'	:('ardrone_autonomy::vector31','ardrone_autonomy::vector31'),
				 'char'			:('std_msgs::UInt8','char'),
				 'bool_t'		:('std_msgs::Int32','bool_t')}




structs = []
print 'Parsing Navdata Struct Contents'
for (struct,name,_) in keys:
	members = []
	rg = re.compile(r'''
		.*?
		typedef\s*struct\s*_?'''+struct+r'''\s*\{
		(?P<inside>.*)
		\}\s*\w*?\s*'''+struct+r'''\s*;
		.*?
		''',re.X|re.DOTALL|re.MULTILINE)
	structcontents = re.search(rg,contents)
	if structcontents:
		print '-- '+name
		allmembers = re.findall(r'(.*?)\s*(\w+(?:\s*?\*)?)\s*(\w+)\s*(?:\[([\w\s*/+-]+)\])?\s*;(.*)',structcontents.groups('inside')[0])
		members = [(t,n,s) for (b,t,n,s,c) in allmembers if 'Deprecated' not in c and '//' not in b]
	structs.append((struct,name,members))





print 'Saving Custom ROS Message Definitions'
for (struct,name,members) in structs:
	with open('../msg/'+name+'.msg','w') as f:
		f.write('Header header\n')
		f.write('float64 drone_time\n')
		for (t,n,s) in members:
			dt = ros_datatypes[t][0]
			dt = dt.rpartition('::')[2].lower()
			if s!='':
				f.write('{0}[] {1}\n'.format(dt,n))
			else:
				f.write('{0} {1}\n'.format(dt,n))




print 'Generating C Source'

items = []
for (struct_type, struct_name, members) in structs:
	item = {}
	item['struct_name'] = struct_name
	item['members'] = []
	for (c_type,name,size) in members:
		member = {}
		member['name'] = name
		member['c_type'] = c_type
		member['include'] = ros_datatypes[c_type][0]
		member['ros_type'] = ros_datatypes[c_type][1]
		member['array_size'] = size if size!='' else None
		item['members'].append(member)
	items.append(item)

template = Template(filename='NavdataMessageDefinitionsTemplate.c');

with open('../src/NavdataMessageDefinitions.h','w') as f:
	f.write(template.render(structs=items))

print 'You should now run `rosmake ardrone_autonomy` to build the custom messages.'
