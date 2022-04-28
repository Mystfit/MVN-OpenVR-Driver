import sys, os, shutil

base_dir = os.getcwd()

for f in os.listdir(os.path.dirname(__file__)):
	ext = os.path.splitext(f)
	valid_extensions = [".obj", ".mtl"]
	if ext[1] in valid_extensions:
		model_folder = os.path.join(base_dir, ext[0])
		if not os.path.isdir(model_folder):
			os.mkdir(model_folder) 
		shutil.copy(f, os.path.join(model_folder, f))
		shutil.copy("puppet_bake.png", os.path.join(model_folder, "puppet_bake.png"))

