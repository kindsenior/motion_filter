# coding: UTF-8
import os
import re
import sys
import yaml
import numpy as np

if len(sys.argv) < 2:
    print "argments: filename robotname(from) robotname(to)"
    sys.exit()

# pseq書き出し
def write_pseq( ofp, pseq ):
    ofp.write("# Body pose sequence format version 1.0 defined by cnoid-Robotics\n\n")
    ofp.write("type: PoseSeq\n")
    ofp.write('name: "'+ pseq["name"] + '"\n')

    ofp.write('targetBody: "' + pseq["targetBody"] + '"\n')
    ofp.write("refs:\n")

    for ref in pseq["refs"]:
        ofp.write("  -\n")
        ofp.write("    time: "+str(ref["time"])+"\n")

        refer = ref["refer"]
        ofp.write("    refer:\n")
        ofp.write("      type: Pose\n")
        ofp.write('      name: "'+refer["name"]+'"\n')

        if "joints" in refer:
            ofp.write("      joints: "+str(refer["joints"])+"\n")
            ofp.write("      q: "+str(refer["q"])+"\n")

        if "ikLinks" in refer:
            ofp.write("      ikLinks:\n")
            for ikLink in refer["ikLinks"]:
                ofp.write("        - \n")
                ofp.write("          name: "+ikLink["name"]+"\n")
                ofp.write("          index: "+str(ikLink["index"])+"\n")
                if "isBaseLink" in ikLink:
                    # ofp.write("          isBaseLink: "+str(ikLink["isBaseLink"])+"\n")
                    ofp.write("          isBaseLink: true\n")
                ofp.write("          translation: "+str(ikLink["translation"])+"\n")
                ofp.write("          rotation: "+str(ikLink["rotation"])+"\n")
                if "isTouching" in ikLink:
                    # ofp.write("          isTouching: "+str(ikLink["isTouching"])+"\n")
                    ofp.write("          isTouching: true\n")
                if "partingDirection" in ikLink:
                    ofp.write("          partingDirection: "+str(ikLink["partingDirection"])+"\n")


            
# 踵関節から爪先関節の位置を計算
def calcTranslation(ikLink):
    translation = ikLink["translation"]
    trans = []
    trans.append([translation[0]]); trans.append([translation[1]]); trans.append([translation[2]])
    trans = np.array(trans);

    rotation = ikLink["rotation"]
    rot = []
    rot.append( rotation[0:3] ); rot.append( rotation[3:6] ); rot.append( rotation[6:9] )
    rot = np.array( rot, dtype=float )

    trans  = np.dot(rot, np.array([[115],[0], [-74]])) + trans
    translation = []
    translation += trans[0].tolist(); translation += trans[1].tolist(); translation += trans[2].tolist();
    return  translation

# 7号機から16号機に変換
def pseq_contert_7_to_16 (fname):
    robotname0 = "HRP2JSK"
    robotname1 = "HRP2JSKNT"
    
    path0 = os.environ.get("HOME")+"/"+os.environ.get("CNOID_WORKSPACE")+"/"+robotname0+"/"+fname+"/"
    ifname = path0 + fname + ".pseq"

    path1 = os.environ.get("HOME")+"/"+os.environ.get("CNOID_WORKSPACE")+"/"+robotname1+"/"+fname+"/"
    ofname = path1 + fname + ".pseq"

    str = open(ifname).read()
    str = str.decode("utf8")
    pseq = yaml.load(str)

    # robotname
    pseq["targetBody"] = robotname1

    for ref in pseq["refs"]:
        refer = ref["refer"]

        # joints, q
        if "joints" in refer:
            joints = refer["joints"]
            q = refer["q"]
            joints_ = []
            if 0 in joints:# rleg
                joints_ += [ 0, 1, 2, 3, 4, 5, 6 ]
                q.insert(7,0)
            if 6 in joints:# lleg
                joints_ += [ 7, 8, 9, 10, 11, 12, 13 ]
                q.insert(joints_.index(13),0)
            if 12 in joints:# chest
                joints_ += [ 14, 15 ]
            if 14 in joints:# neck
                joints_ += [ 16, 17 ]
            if 16 in joints:# rarm
                joints_ += [ 18, 19, 20, 21, 22, 23, 24, 25 ]
            if 24 in joints:# larm
                joints_ += [ 26, 27, 28, 29, 30, 31, 32, 33 ]
            refer["joints"] = joints_[:]
            refer["q"] = q[:]

        # ikLink
        if "ikLinks" in refer:
            for ikLink in refer["ikLinks"]:
                if ikLink["name"] == "RLEG_JOINT5":
                    # 爪先関節のikLinkを追加
                    toeIkLink = {"index":7, "name":"RLEG_JOINT6", "translation":calcTranslation(ikLink), "partingDirection":[0,0,1] ,"rotation": ikLink["rotation"][:] }
                    if "isTouching" in ikLink:
                        toeIkLink["isTouching"] = "true"
                    refer["ikLinks"].append(toeIkLink)

                if ikLink["name"] == "LLEG_JOINT5":
                    ikLink["index"] = 13
                    # 爪先関節のikLinkを追加
                    toeIkLink = {"index":14, "name":"LLEG_JOINT6", "translation":calcTranslation(ikLink), "partingDirection":[0,0,1] ,"rotation": ikLink["rotation"][:] }
                    if "isTouching" in ikLink:
                        toeIkLink["isTouching"] = "true"
                    refer["ikLinks"].append(toeIkLink)

                if ikLink["name"] == "RARM_JOINT6":
                    ikLink["index"] = 25

                if ikLink["name"] == "LARM_JOINT6":
                    ikLink["index"] = 38

    # 保存
    if not os.path.isdir(path1): os.mkdir(path1)
    ofp = open(ofname,"w")
    write_pseq( ofp, pseq );
    # str = yaml.dump(pseq, encoding="utf8", allow_unicode=True)
    # ofp.write(str)
    ofp.close()        

pseq_contert_7_to_16(sys.argv[1]);
# pseq_convert(sys.argv[1], sys.argv[2], sys.argv[3])

