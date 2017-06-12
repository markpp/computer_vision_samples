import os

root_data_path = "/Users/markpp/Desktop/code/data/DanpoData/"
# root_data_path = "/home/jacob/data/DanpoData/"

os.system("echo 'Executing annotation_prep..'")
# os.system("./annotation_prep/build/annotation_prep " + root_data_path + "CENTER_A " + "_C_A")
# os.system("./annotation_prep/build/annotation_prep " + root_data_path + "CENTER_B " + "_C_B")
# os.system("./annotation_prep/build/annotation_prep " + root_data_path + "RIGHT_A " + "_R_A")
# os.system("./annotation_prep/build/annotation_prep " + root_data_path + "RIGHT_B " + "_R_B")

os.system("echo 'Executing label_point_cloud..'")
# os.system("./label_point_cloud/build/label_point_cloud " + root_data_path + "CENTER_A " + "_C_A")
# os.system("./label_point_cloud/build/label_point_cloud " + root_data_path + "CENTER_B " + "_C_B")
# os.system("./label_point_cloud/build/label_point_cloud " + root_data_path + "RIGHT_A " + "_R_A")
# os.system("./label_point_cloud/build/label_point_cloud " + root_data_path + "RIGHT_B " + "_R_B")

os.system("echo 'Executing filter_point_cloud..'")
os.chdir("filter_point_cloud/build/")
os.system("make")
os.system("ls")
os.system("./filter_point_cloud " + root_data_path + "CENTER_A " + "_C_A")
os.system("./filter_point_cloud " + root_data_path + "CENTER_B " + "_C_B")
os.system("./filter_point_cloud " + root_data_path + "RIGHT_A " + "_R_A")
os.system("./filter_point_cloud " + root_data_path + "RIGHT_B " + "_R_B")
