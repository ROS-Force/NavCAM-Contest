#!/usr/bin/python

import os

fileIds = ["1wWFqIH3vNQWhhCheoHg_v5sjG2JJ25fB"
,"1RQRGJVmPDm2MklVClgrIHBatX2bDgLli"
,"1HjJBIVyasTYZH9yxRU1CV7t9VUQfWQvg"
,"1h53s9DMv9gypDkTHak1QDFRpjIY_FsXU"
,"1RW_vdkntVxU9zgOdu_ps7GYfEuSMlqVm"]


download_url = "wget --load-cookies /tmp/cookies.txt \"https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate \'https://docs.google.com/uc?export=download&id={0}\' -O- | sed -rn \'s/.*confirm=([0-9A-Za-z_]+).*/\\1\\n/p\')&id={0}\" -O {1} && rm -rf /tmp/cookies.txt"

for i in range(len(fileIds)):
    os.system(download_url.format(fileIds[i],"../data/challenge1_2/" + str(i+1) + "meter.bag"))