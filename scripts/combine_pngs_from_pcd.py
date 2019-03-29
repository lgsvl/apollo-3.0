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

max_cols = 0
max_dirname = ""
for dirname, paths in image_paths_dic.items():
	if len(paths) > max_cols:
		max_cols = len(paths)
		max_dirname = dirname
rows = len(image_paths_dic)

print("rows", rows, "maximum cols", max_cols)

# create empty image with maximum row, cols

images = [[Image.open(path) for path in sorted(image_paths_dic[key])] for key in sorted(image_paths_dic.keys())]

total_width = max_cols * 1024
total_height = rows * 1024

new_im = Image.new('RGB', (total_width, total_height))

for row in range(rows):
	x_offset = 0
	for col in range(max_cols):
		try:
			new_im.paste(images[row][col], (x_offset, row * images[row][col].size[1]))
			x_offset += images[row][col].size[0]
		except:
			continue

os.chdir(map_root_folder)

new_im = new_im.transpose(Image.FLIP_TOP_BOTTOM)
new_im.save('background.png')
new_im.save('background.jpg')

# Compute Origin for HDMapTool and GPSDevice
file_name = sorted(image_paths_dic[max_dirname])[max_cols//2].split('/')[-1]
origin_easting = int(file_name.split('.')[0])*128

if max_cols%2 != 0:
	origin_easting += 64

origin_northing = int(sorted(image_paths_dic.keys())[rows//2].split('/')[-1]) * 128
if rows%2 != 0:
	origin_northing += 64
print("origin_northing:", origin_northing, "origin_easting:", origin_easting)

# Compute Scale for Unity
scale_x = rows*128 / 10.
scale_y = max_cols*128 / 10.
print("The scale for X and Z of the combined png in Unity is:", scale_x, scale_y)
