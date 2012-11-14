import re

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

ros_datatypes = {'uint32_t':'uint32',
				 'uint16_t':'uint16',
				 'uint8_t' :'uint8',
				 'int32_t':'int32',
				 'int16_t':'int16',
				 'int8_t' :'int8',
				 'float64_t':'float64',
				 'float32_t':'float32',
				 'matrix33_t':'ardrone_autonomy::matrix33',
				 'vector31_t':'ardrone_autonomy::vector31',
				 'vector21_t':'ardrone_autonomy::vector21',
				 'velocities_t':'ardrone_autonomy::vector31',
				 'char':'uint8',
				 'bool_t':'int32'}

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
		allmembers = re.findall(r'\s*(\w+(?:\s*?\*)?)\s*(\w+)\s*(?:\[(\w+)\])?\s*;(.*)',structcontents.groups('inside')[0])
		members = [(t,n,s) for (t,n,s,c) in allmembers if 'Deprecated' not in c]
	structs.append((struct,name,members))


print 'Saving Custom ROS Message Definitions'
for (struct,name,members) in structs:
	with open('../msg/'+name+'.msg','w') as f:
		f.write('Header header\n')
		f.write('float64 drone_time')
		for (t,n,s) in members:
			dt = ros_datatypes[t]
			if s!='':
				f.write('{0}[] {1}\n'.format(dt,n))
			else:
				f.write('{0} {1}\n'.format(dt,n))

print 'Saving Struct Preprocessor File'
with open('../src/NavdataMessageDefinitions.h','w') as f:
	for (struct,name,members) in structs:
		f.write('NavdataStructStart({0},{1})\n'.format(struct,name))
		for (t,n,s) in members:
			dt = ros_datatypes[t]
			if s!='':
				f.write('NavdataStructArray({0},{1},{2},{3},{4},{5})\n'.format(struct,name,t,dt,s,n))
			else:
				f.write('NavdataStructMember({0},{1},{2},{3},{4})\n'.format(struct,name,t,dt,n))
		f.write('NavdataStructEnd({0},{1})\n\n'.format(struct,name))

print 'You should now run `rosmake ardrone_autonomy` to build the custom messages.'
