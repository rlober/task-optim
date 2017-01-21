import re
import os
import subprocess
import time
import shlex

def combineVideos(test_dir):
    ffmpeg_args = """ffmpeg -y -i """ + test_dir + """/original.mp4 -i """ + test_dir + """/optimal.mp4 -filter_complex "[0:v][1:v]hstack=inputs=2[v]" -map "[v]" -ac 2 """+test_dir+"""/temp_0.mp4 \
    && \
    ffmpeg -i """ + test_dir + """/temp_0.mp4 -y -vf drawtext="text='ORIGINAL':\
    fontcolor=white:\
    fontsize=48:\
    box=1:\
    boxcolor=black@0.8:\
    boxborderw=10:\
    x=(main_w)/4:\
    y=main_h-(text_h*2)" -codec:a copy """ + test_dir + """/temp_1.mp4 \
    && \
    ffmpeg -i """ + test_dir + """/temp_1.mp4 -y -vf drawtext="text='OPTIMAL':\
    fontcolor=white:\
    fontsize=48:\
    box=1:\
    boxcolor=black@0.8:\
    boxborderw=10:\
    x=(main_w)*3/4:\
    y=main_h-(text_h*2)" -codec:a copy """ + test_dir + """/optimal_and_original.mp4 \
    && \
    rm """+test_dir+"""/temp_0.mp4 """+test_dir+"""/temp_1.mp4
    """

    ffmpeg_args = shlex.split(ffmpeg_args)
    print(ffmpeg_args)
    subprocess.Popen(ffmpeg_args)#, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("View video at:\n", test_dir + "/optimal_and_original.mp4")



root_tests_dir = os.path.expanduser("~") + "/Optimization_Tests/rand_right_hand_target_tests/bo/"
test_name = 'OneComWaypointStaticTest'
dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]
test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match(test_name+'.*', d)])

for i, t in enumerate(test_dirs[0:1]):
    print('Recording simulation for test', i+1, 'of', len(test_dirs), '.', (i+1)/len(test_dirs)*100, "% complete.")
    try:
        combineVideos(t)
    except:
        print("Couldn't format videos from:\n", t)
