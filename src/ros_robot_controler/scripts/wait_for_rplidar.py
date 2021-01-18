import subprocess


topic_found = False

while topic_found == False:

    try:
        topics = subprocess.check_output("rostopic list", shell=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as e:
        print('error getting topics')
        topics = ""

    t = topics.splitlines()
    for topic in t:
        if topic == '/scan':
           print('topic found')
           topic_found = True
