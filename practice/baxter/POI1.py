#-*-coding:UTF-8-*- 
import json
import sys
import requests  #导入requests库，这是一个第三方库，把网页上的内容爬下来用的
ty=sys.getfilesystemencoding()  #这个可以获取文件系统的编码形式
import time
lat_1=24.390894
lon_1=102.174112
lat_2=26.548645
lon_2=103.678942   #坐标范围
las=1  #给las一个值1
ak='9s5GSYZsWbMaFU8Ps2V2VWvDlDlqGaaO'
#push=r'D:\python\zwzwlast.txt'
#push = r'/home/tianyu/practice/POI23.txt'
#我们把变量都放在前面，后面就不涉及到变量了，如果要爬取别的POI，修改这几个变量就可以了，不用改代码了。


print (time.time())  #相较于python2.7，,python3print 需要加括号。
print ('开始')
urls=[] #声明一个数组列表
lat_count=int((lat_2-lat_1)/las+1)
lon_count=int((lon_2-lon_1)/las+1)
for lat_c in range(0,lat_count):
    lat_b1=lat_1+las*lat_c
    for lon_c in range(0,lon_count):
        lon_b1=lon_1+las*lon_c
        for i in range(0,20):
            page_num=str(i)
            url='http://api.map.baidu.com/place/v2/search?query=火锅& bounds='+str(lat_b1)+','+str(lon_b1)+','+str(lat_b1+las)+','+str(lon_b1+las)+'&page_size=20&page_num='+str(page_num)+'&output=json&ak='+ak
            urls.append(url)
#urls.append(url)的意思是，将url添加入urls这个列表中。

#f=open(r'D:\python\kunmingxuexiao.txt','a',encoding='utf-8')
f=open(r'/home/tianyu/practice/POI222.xlsx','a',encoding='utf-8')

print ('url列表读取完成')

for url in urls:
    time.sleep(10) #为了防止并发量报警，设置了一个10秒的休眠。
    html=requests.get(url)#获取网页信息
    data=html.json()#获取网页信息的json格式数据
    for item in data['results']:
        jname=item['name']
        jlat=item['location']['lat']
        jlon=item['location']['lng']
        jadd=item['address']
        j_str=jname+','+str(jlat)+','+str(jlon)+','+jadd+'\n'
        f.write(j_str)
    print (time.time())
f.close()
print ('完成')    
