#pragma once

#include <list>
#include <stdio.h>
#include <cmath>
#include <string>
#include <vector>
#include <assert.h>
#include <map>

#include "autoware_msgs/DetectedObjectArray.h"

class Tracker;

class TrackedObj {
public:
    autoware_msgs::DetectedObject prev_obj;
    autoware_msgs::DetectedObject obj;
    int obj_id;
    int id;
    int appearCnt;
    int disappearCnt;
    int trackedCnt;
    Tracker* tracker;
    enum TrackingStatus {
        INSPECTING,
        TRACKING,
        LOSTING
    };

    TrackingStatus status;
    void init(const autoware_msgs::DetectedObject& obj, int obj_id, Tracker* tracker);
    void disappear();
    void appear(const autoware_msgs::DetectedObject& obj, int obj_id);
    void release();
    void smooth_update();

    // used for smoothing the object
    // void predict() {}
    // void correct(const autoware_msgs::DetectedObject& obj) {}

    ~TrackedObj() {}
};

class Tracker {
public:
    friend class TrackedObj;
    float dist_thr;
    int appear_thr;
    int disappear_thr;
protected:
    int object_index;
    std::list<TrackedObj*> tracked_objects;
public:
    Tracker();
    void init();
    bool similar(const autoware_msgs::DetectedObject& a, const autoware_msgs::DetectedObject& b);
    int track(const std::vector<autoware_msgs::DetectedObject> &objects,  std::vector<autoware_msgs::DetectedObject> &out_objects);
    ~Tracker();
};
