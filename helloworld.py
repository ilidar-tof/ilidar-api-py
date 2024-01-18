from ilidar import iTFS

ilidar = iTFS()

while True:
    str = input()

    if str[0] == 's' or str[0] == 'S':
        break

ilidar.Try_exit()
