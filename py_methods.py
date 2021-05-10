import os


def ply2txt():
    fileNum = 0
    while fileNum != 9:
        filePath = '.\\data\\'
        ls_path = os.listdir(filePath)
        fileNum = len(ls_path)
        for filename in ls_path:
            if filename[-3:] == "txt":
                os.remove(filePath + filename)
            if filename[-5:] == "A.ply":
                with open(filePath + filename, 'r+', encoding='utf-8') as file_read:
                    with open(filePath + filename[:-3] + "txt", 'w', encoding='utf-8') as file_write:
                        for line in file_read.readlines():
                            if (line[0].isdigit() or line[0] == "-") and line[:7] != "0 0 0 1":
                                file_write.write(line)
                print("CONVERT: " + filename[:-3] + "txt")


def merge_txt():
    idx = 0
    filePath = '.\\data\\'
    filename_out = 'VRR_preprocessed.txt'
    ls_path = os.listdir(filePath)
    with open(filePath + filename_out, 'w', encoding='utf-8') as file_write:
        for filename in ls_path:
            if filename[-5:] == "A.txt":
                with open(filePath + filename, 'r+', encoding='utf-8') as file_read:
                    for line in file_read.readlines():
                        file_write.write(line[:-2] + str(idx) + "\n")
                    idx += 1
    print("MERGE: VRR_preprocessed.txt")
