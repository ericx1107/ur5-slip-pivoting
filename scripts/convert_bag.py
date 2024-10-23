# import bagpy

import subprocess, yaml

info_dict = yaml.safe_load(subprocess.Popen(['rosbag', 'info', '--yaml', '/home/acrv/eric_ws/bagfiles/long_pivot_1.bag'], 
                                       stdout=subprocess.PIPE).communicate()[0])

print(info_dict)

# from bagpy import bagreader
# b = bagreader('/home/acrv/eric_ws/bagfiles/long_pivot_1.bag')
# b.topic_table
# print(b.topic_table)
# data = b.message_by_topic('/tf')
