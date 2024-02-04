import rosbag, sys, csv
import time
import os #for file management make directory
import glob

bagfiles = glob.glob('../bag/*.bag')
print("Number of bag files found in the repository: " + str(len(bagfiles)))

for bagfile in bagfiles:
  print("Reading the file " + bagfile)

  bag = rosbag.Bag(bagfile)
  bag_contents = bag.read_messages()
  bag_name = bag.filename
  print("- bag filename: " + bag_name)

  csv_filename = bag_name.strip("bag") + "csv"
  print("- csv filename: " + csv_filename)

  topics_list = []
  for topic, msg, t in bag_contents:
    if topic not in topics_list:
      topics_list.append(topic)
  print("- list of topics:")
  print(topics_list)

  if len(topics_list) != 1:
    print("ERROR: only expected a single topic (expected: 1 vs " + \
          str(len(topics_list)) + "; /*/motors_data)")
    continue
  
  if not "motors_data" in str(topics_list[0]):
    print("ERROR: invalid topic (expected: /*/motors_data vs " + \
          str(topics_list[0]) + ")")
    continue

  with open(csv_filename, 'w') as csv_file:
    filewriter = csv.writer(csv_file, delimiter = ',')
    first_iteration = True

    for subtopic, msg, t in bag.read_messages(topics_list[0]):
      msg_str = str(msg)
      msg_list = msg_str.split("\n")

      instantaneous_list_data = []
      for name_value_pair in msg_list:
        split_pair = name_value_pair.split(":")
        
        for i in range(len(split_pair)):
          split_pair[i] = split_pair[i].strip()

        instantaneous_list_data.append(split_pair)
      
      if first_iteration:
        headers = ["rosbagTimestamp"]
        for pair in instantaneous_list_data:
          headers.append(pair[0])
        
        filewriter.writerow(headers)
        first_iteration = False

      values = [str(t)]
      for pair in instantaneous_list_data:
        if (len(pair) > 1):
          values.append(pair[1])
      
      filewriter.writerow(values)

  bag.close()
