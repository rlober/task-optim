import re
import os
import subprocess
import time
import shlex

def combineVideos(test_dir):
    if (os.path.exists(os.path.join(test_dir, "original.mp4"))) and (os.path.exists(os.path.join(test_dir, "optimal.mp4"))):
        ffmpeg_args = """ffmpeg -y -i """ + test_dir + """/original.mp4 -i """ + test_dir + """/optimal.mp4 -filter_complex "[0:v][1:v]hstack=inputs=2[v]" -map "[v]" -ac 2 """+test_dir+"""/temp_0.mp4"""

        ffmpeg_args = shlex.split(ffmpeg_args)
        proc_1 = subprocess.Popen(ffmpeg_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        proc_1.wait()

        ffmpeg_args = """ffmpeg -i """ + test_dir + """/temp_0.mp4 -y -vf drawtext="text='ORIGINAL':\
        fontcolor=white:\
        fontsize=48:\
        box=1:\
        boxcolor=black@0.8:\
        boxborderw=10:\
        x=(main_w)/4:\
        y=main_h-(text_h*2)" -codec:a copy """ + test_dir + """/temp_1.mp4"""

        ffmpeg_args = shlex.split(ffmpeg_args)
        proc_1 = subprocess.Popen(ffmpeg_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        proc_1.wait()

        ffmpeg_args = """ffmpeg -i """ + test_dir + """/temp_1.mp4 -y -vf drawtext="text='OPTIMAL':\
        fontcolor=white:\
        fontsize=48:\
        box=1:\
        boxcolor=black@0.8:\
        boxborderw=10:\
        x=(main_w)*3/4:\
        y=main_h-(text_h*2)" -codec:a copy """ + test_dir + """/temp_2.mp4"""

        ffmpeg_args = shlex.split(ffmpeg_args)
        proc_1 = subprocess.Popen(ffmpeg_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        proc_1.wait()

        video_path = test_dir + "/optimal_and_original.mp4"


        ffmpeg_args = """ffmpeg -i """ + test_dir + """/temp_2.mp4 -y -vf drawtext="text='"""+video_path+"""':\
        fontcolor=white:\
        fontsize=24:\
        box=1:\
        boxcolor=black@0.8:\
        boxborderw=10:\
        x=(main_w)/2:\
        y=(text_h*2)" -codec:a copy """ + test_dir + """/optimal_and_original.mp4"""

        ffmpeg_args = shlex.split(ffmpeg_args)
        proc_1 = subprocess.Popen(ffmpeg_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        proc_1.wait()

        ffmpeg_args = """rm """+test_dir+"""/temp_0.mp4 """+test_dir+"""/temp_1.mp4 """+test_dir+"""/temp_2.mp4"""

        ffmpeg_args = shlex.split(ffmpeg_args)
        proc_1 = subprocess.Popen(ffmpeg_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        proc_1.wait()

        print("View video at:\n", video_path)
        return video_path
    else:
        print("ERROR - Test at:", test_dir, "is missing videos.")


def concatenateTestVideos(root_tests_dir, destination_dir, video_name):
    test_name = 'OneComWaypointStaticTest'
    dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]
    test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match(test_name+'.*', d)])

    errored_dirs = []

    cat_file_path = os.path.join(root_tests_dir,"ffmpeg_cat_list.txt")
    cat_file = open(cat_file_path, 'wb')

    for i, t in enumerate(test_dirs):
        vid_path = os.path.join(t,"optimal_and_original.mp4")
        if os.path.exists(vid_path):
            # print('Processing', i+1, 'of', len(test_dirs), '-', (i+1)/len(test_dirs)*100, "% complete.")
            line_text = "file '" + vid_path + "'"
            cat_file.write(vid_path)


        else:
            print("Couldn't process video from", t, "-- because it doesn't exist.")
            errored_dirs.append(t)

    cat_file.close()


    final_vid_path = os.path.join(destination_dir, video_name)

    args = "ffmpeg -f concat -i "+cat_file+" -c copy "+final_vid_path
    args = shlex.split(args)
    proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    proc.wait()

    return errored_dirs

def copyAllTestVideos(root_tests_dir, destination_dir, videos_to_copy=["optimal_and_original"]):
    test_name = 'OneComWaypointStaticTest'
    dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]
    test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match(test_name+'.*', d)])

    for i, (t, v) in enumerate(zip(test_dirs,videos_to_copy)):
        vid_path = os.path.join(t,v+".mp4")
        if os.path.exists(vid_path):
            print('Copying', i+1, 'of', len(test_dirs), '-', (i+1)/len(test_dirs)*100, "% complete.")
            args = "cp " + vid_path + " " + os.path.join(dropbox_dir, str(i).zfill(3)+".mp4")
            args = shlex.split(args)
            proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            proc.wait()


root_tests_dir = os.path.expanduser("~") + "/Optimization_Tests/rand_right_hand_target_tests/bo/"
dropbox_dir = os.path.expanduser("~") + "/Dropbox/RandReachVideos/"

combineVideos(root_tests_dir)
concatenateTestVideos(root_tests_dir, dropbox_dir, "RandReachVideos.mp4"):



# test_name = 'OneComWaypointStaticTest'
# dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]
# test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match(test_name+'.*', d)])
#
# for i, t in enumerate(test_dirs):
#     print('Copying', i+1, 'of', len(test_dirs), '-', (i+1)/len(test_dirs)*100, "% complete.")
#     args = """cp """+t+"""/optimal_and_original.mp4 """+dropbox_dir+"""/"""+str(i).zfill(3)+""".mp4"""
#     args = shlex.split(args)
#     proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#     proc.wait()
#     # print('Recording simulation for test', i+1, 'of', len(test_dirs), '.', (i+1)/len(test_dirs)*100, "% complete.")
#     # try:
#     #     combineVideos(t)
#     # except:
#     #     print("Couldn't format videos from:\n", t)
