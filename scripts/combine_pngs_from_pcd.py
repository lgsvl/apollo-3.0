import sys, os
from PIL import Image
from collections import defaultdict

"""Please place this script inside the lossy_map folder"""
images_path = './image'
map_root_folder = '..'

def find_all_images(topdir, extension):
	# return the list of all proto paths

	paths_dic = defaultdict(list)

	def step(ext, dirname, names):
		ext = ext.lower()

		for name in names:
			if name.lower().endswith(ext): # do not convert Racobit due to naming conflict and we are not using this radar
				# print(os.path.join(dirname, name))
				value = os.path.join(dirname, name)
				paths_dic[dirname].append(value)

	os.path.walk(topdir, step, extension)
	return paths_dic

os.chdir(images_path)
image_paths_dic = find_all_images('.', '.png')

print(image_paths_dic)
print

max_dirname = ""
min_northing = None # this northing is before multiply 128
max_northing = None
min_easting = None
max_easting = None

images_dic = {}
for dirname, paths in image_paths_dic.items():
	# update min/max northing number
	northing = int(dirname.split('/')[-1])
	print("row number is ", northing)

	if min_northing is None or northing < min_northing:
		min_northing = northing
	if max_northing is None or northing > max_northing:
		max_northing = northing

	temp_min_easting, temp_max_easting = int(min(paths).split('/')[-1].split('.')[0]), int(max(paths).split('/')[-1].split('.')[0])
	print(temp_min_easting, temp_max_easting)

	if min_easting is None or temp_min_easting < min_easting:
		min_easting = temp_min_easting
	if max_easting is None or temp_max_easting > max_easting:
		max_easting = temp_max_easting

	images_dic[northing] = {}
	for path in paths:
		easting = int(path.split('/')[-1].split('.')[0])
		images_dic[northing][easting] = Image.open(path)

num_northings = max_northing - min_northing + 1
num_eastings = max_easting - min_easting + 1

print("min_easting is", min_easting, "max_easting is", max_easting)
print("min_northing is", min_northing, "max_northing is", max_northing)
print("num_northings", num_northings, "num_eastings", num_eastings)

# create empty image with maximum row, cols
total_width = num_eastings * 1024
total_height = num_northings * 1024

new_im = Image.new('RGB', (total_width, total_height))

for northing in range(min_northing, max_northing+1):
	for easting in range(min_easting, max_easting+1):
		try:
			new_im.paste(images_dic[northing][easting], ((easting - min_easting) * images_dic[northing][easting].size[0], (northing - min_northing) * images_dic[northing][easting].size[1]))
		except:
			continue

os.chdir(map_root_folder)

new_im = new_im.transpose(Image.FLIP_TOP_BOTTOM)
new_im.save('background.png')
new_im.save('background.jpg')

# Compute Origin for HDMapTool and GPSDevice
origin_easting = (min_easting + num_eastings // 2) * 128

if num_eastings%2 != 0:
	origin_easting += 64

origin_northing = (min_northing + num_northings // 2) * 128
if num_northings%2 != 0:
	origin_northing += 64
print("origin_northing:", origin_northing, "origin_easting:", origin_easting)

# Compute Scale for Unity
scale_x = num_northings*128 / 10.
scale_y = num_eastings*128 / 10.
print("The scale for X and Z of the combined png in Unity is:", scale_x, scale_y)
