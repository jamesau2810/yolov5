def waypoint_file_read(filePath):
    db =open(filePath)
    myline = db.readlines()
    dataLine = myline[1:]
    words = [i.rstrip("\n").split("\t")  for i in dataLine]
    points = [[i[0],i[8],i[9],i[10]]  for i in words if i[3]==16]

    return words,points
w,t = waypoint_file_read('/Users/jamesau/Documents/GitHub/yolov5/Trial002_waypoints.txt')
print(w)
for i in w:
    print(len(i))
# 
# myline = db.readline()
# arr = []
# while myline:
#     # print(myline)
#     myline = db.readline()
#     xp = myline.split("\t")
#     print(xp)
#     arr.append(myline)
# db.close()
# print(arr)
