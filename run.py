import os
import py_methods as pm

main = ""
start = '.\\'
target_filename = 'geo1101.exe'
for relPath, dirs, files in os.walk(start):
    if target_filename in files:
        full_path = os.path.join(start, relPath, target_filename)
        main = os.path.normpath(os.path.abspath(full_path))
print("RUN: " + target_filename + " from " + main)
os.system(main)

pm.ply2txt()
pm.merge_txt()
