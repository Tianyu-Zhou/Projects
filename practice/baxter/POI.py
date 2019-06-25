# -*- coding:utf-8 -*
import json  #导入json库
import os
import urllib2 #导入urllib2库
import sys #我知道了，这个东东是为了把默认编码从ascii转到uft-8。
reload(sys)
sys.setdefaultencoding('utf-8')

lat_1=24.390894
lon_1=102.174112
lat_2=26.548645
lon_2=103.678942   #坐标范围
las=1  #给las一个值1
ak='9s5GSYZsWbMaFU8Ps2V2VWvDlDlqGaaO'
keyword='中学'
#push=r'D:\python\12345.txt'
push = r'/home/tianyu/practice/POI123.txt'
#我们把变量都放在前面，后面就不涉及到变量了，如果要爬取别的POI，修改这几个变量就可以了，不用改代码了。
f=open(push,'a')
lat_count=int((lat_2-lat_1)/las+1)
lon_count=int((lon_2-lon_1)/las+1)
for lat_c in range(0,lat_count):
    lat_b1=lat_1+las*lat_c
    for lon_c in range(0,lon_count):
        lon_b1=lon_1+las*lon_c
        for i in range(0,20):
            page_num=str(i)
            url='http://api.map.baidu.com/place/v2/search?query=中学& bounds='+str(lat_b1)+','+str(lon_b1)+','+str(lat_b1+las)+','+str(lon_b1+las)+'&page_size=20&page_num='+str(page_num)+'&output=json&ak='+ak
            response = urllib2.urlopen(url)
            data=json.load(response)
            for item in data['results']:
                jname=item['name']
                jlat=item['location']['lat']
                jlon=item['location']['lng']
                jadd=item['address']
                j_str=jname+','+str(jlat)+','+str(jlon)+','+jadd+'\n'
                f.write(j_str)
f.close()
