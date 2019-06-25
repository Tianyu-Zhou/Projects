## This is an example1! ##
## users = ['admin','b','c','d','e']
## onlines = ['admin','b','c']
## if onlines:
##     for online in onlines:
##         if online in users:
##             if onlines == 'admin':
##                 print("HELLO ADMIN.")
##             else:
##                 print("hello" + online + ".")
##         else:
##             print("This user is not exist!")
## else:
##     print("We need to find some users!")
#####################################################
## This is the example2! ##
current_users = ['A','b','C','d','e']
new_users = ['a','c','f','g','h']
current_user_news = []
if new_users:
    for current_user in current_users:
        current_user_new = current_user.lower()
        current_user_news.append(current_user_new)
    for new_user in new_users:
        if new_user.lower() in current_user_news:
            print("Need input another user name!")
        else:
            print("This user name is unused!")
else:
    print("HEHE!")
