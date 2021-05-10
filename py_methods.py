import os


def ply2txt():
    fileNum = 0
    while fileNum != 9:
        filePath = '.\\data\\'
        ls_path = os.listdir(filePath)
        fileNum = 0

        for filename in ls_path:
            if filename[-3:] == "txt":
                os.remove(filePath + filename)
            else:
                fileNum += 1

        ls_path = os.listdir(filePath)
        for filename in ls_path:
            if filename[-5:] == "A.ply":
                with open(filePath + filename, 'r+', encoding='utf-8') as file_read:
                    with open(filePath + filename[:-3] + "txt", 'w', encoding='utf-8') as file_write:
                        for line in file_read.readlines():
                            if (line[0].isdigit() or line[0] == "-") and line[:7] != "0 0 0 1":
                                if filename.split("_")[1] == "rest":
                                    if filename.split("_")[2] == "archi":
                                        file_write.write(line[:-2] + "2" + "\n")
                                    elif filename.split("_")[2] == "nonarchi":
                                        file_write.write(line[:-2] + "3" + "\n")
                                elif filename.split("_")[1] == "roof":
                                    file_write.write(line[:-2] + "0" + "\n")
                                elif filename.split("_")[1] == "ground":
                                    file_write.write(line[:-2] + "1" + "\n")

                print("CONVERT: " + filename[:-3] + "txt")


def merge_txt():
    filePath = '.\\data\\'
    filename_out = 'VRR_preprocessed.txt'
    ls_path = os.listdir(filePath)
    with open(filePath + filename_out, 'w', encoding='utf-8') as file_write:
        for filename in ls_path:
            if filename[-5:] == "A.txt":
                with open(filePath + filename, 'r+', encoding='utf-8') as file_read:
                    for line in file_read.readlines():
                        file_write.write(line)
    print("MERGE: VRR_preprocessed.txt")
