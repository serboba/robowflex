import xml.etree.ElementTree as ET
import os
import sys


def parse_urdf_file(root):

    for group in root.findall('joint'):
        joint_name = group.get('name')
        if(joint_name=="goal"):
            goal_pos = group.find("origin").get("xyz") +" "+ group.find("origin").get("rpy")
            print(goal_pos)
            return goal_pos

    return -1


def parse_srdf_file(root):
    groups = []
    index = 0
    for group in root.findall('group'):
        groups_name = group.get('name')
        jointcount = len(group.getchildren())
        gr_n_dim = groups_name + ',' + str(jointcount) + ',' + str(index)
        index += jointcount
        groups.append(gr_n_dim)
    return groups

def translate_into_txt(filename):
    filename1 = 'envs/'+filename
    srdf_name = filename1 +'.srdf'
    urdf_name = filename1 +'.urdf'
    print(srdf_name)
    print(urdf_name)
    print(os.getcwd())
    txt_file = []

    res = parse_srdf_file(ET.parse(srdf_name).getroot())
    for r in res:
        txt_file.append(r)
    txt_file.append('-')

    txt_file.append(parse_urdf_file(ET.parse(urdf_name).getroot()))
    txt_name = filename + '.txt'
    write_into_txt(txt_file,txt_name)

def write_into_txt(file,name):
    os.getcwd()
    direc = "txt_files/"
    name_ = os.path.join(direc,name)
    with open(name_, 'w') as f:
        for item in file:
            f.write("%s\n" % item)


if len(sys.argv) == 1:
     sys.exit("NOT ENOUGH ARGS")


os.chdir("../../..")
os.chdir(os.getcwd()+"/src/robowflex/robowflex_dart/include/io/")

translate_into_txt(str(sys.argv[1]))


# def main():
#     translate_into_txt("maze2")
#
# if __name__ == '__main__':
#      main()
#
