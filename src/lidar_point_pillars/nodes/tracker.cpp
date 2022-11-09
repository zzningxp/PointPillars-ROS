#include "tracker.h"

bool Tracker::similar(const autoware_msgs::DetectedObject& a, const autoware_msgs::DetectedObject& b) {
    float d_x = a.pose.position.x - b.pose.position.x;
    float d_y = a.pose.position.y - b.pose.position.y;
    float d_z = a.pose.position.z - b.pose.position.z;

    float dist = sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
    return (dist < dist_thr);
}

Tracker::Tracker() {
    object_index = 0;
    dist_thr = 5;
    appear_thr = 3;
    disappear_thr = 3;
}

int Tracker::track(const std::vector<autoware_msgs::DetectedObject> &objects, std::vector<autoware_msgs::DetectedObject> &out_objects) {
    if (appear_thr <= 0) {
        out_objects = objects;
    }
    else {
        std::list<TrackedObj*>::iterator ii;

        for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
            TrackedObj* tobj = *ii;
            tobj->prev_obj = tobj->obj;
            tobj->obj_id = -1;
        }

        for (int i = 0; i < objects.size(); i++) {
            const autoware_msgs::DetectedObject& obj = objects[i];
            bool isnew = true;

            for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
                TrackedObj* tobj = *ii;
                if (similar(obj, tobj->obj)) {
                    isnew = false;
                    tobj->appear(obj, i);
                    // tobj->correct(obj);
                    break;
                }
            }

            if (isnew) {
                TrackedObj* new_obj = new TrackedObj();
                new_obj->init(obj, i, this);
                tracked_objects.push_back(new_obj);
                //printf("create Track Object with trackid %d\n", new_tobj.id);
            }
        }

        // Remove disappear objects
        for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
            TrackedObj* tobj = *ii;
            if (-1 == tobj->obj_id) {
                tobj->disappear();
                // tobj->predict();
            }
        }

        // Remove lost objects
        for (ii = tracked_objects.begin(); ii != tracked_objects.end();) {
            TrackedObj* tobj = *ii;
            if (tobj->status == TrackedObj::LOSTING) {
                tobj->release();
                delete tobj;
                ii  = tracked_objects.erase(ii );
            } else {
                ++ii ;
            }
        }

        for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
            TrackedObj* tobj = *ii;
            tobj->trackedCnt += 1;
            // tobj->smooth_update();
        }

        out_objects.clear();

        for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
            TrackedObj* tobj = *ii;
            if (tobj->status == TrackedObj::TRACKING) {
                autoware_msgs::DetectedObject& obj = tobj->obj;
                obj.id = tobj->id;
                // tobj->output(&obj);
                out_objects.push_back(obj);
            }
        }

        std::cout << "tracked_objects size: " << tracked_objects.size() << "\n";
        for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
            TrackedObj* tobj = *ii;
            std::cout << "Tracked Id: " << tobj->id << " status: " << tobj->status << 
                " appearCnt: " << tobj->appearCnt << 
                " disappearCnt: " << tobj->disappearCnt <<
                " trackedCnt: " << tobj->trackedCnt << "\n";
        }
    }

    return 0;
}

Tracker::~Tracker() {
    std::list<TrackedObj*>::iterator ii = tracked_objects.begin();
    for (; ii != tracked_objects.end(); ++ii ) {
       TrackedObj* tobj = *ii ;
       delete tobj;
    }
}

void TrackedObj::init(const autoware_msgs::DetectedObject &obj, int obj_id, Tracker *tracker) {
    this->obj = obj;
    this->obj_id = obj_id;
    this->id = -1;
    this->appearCnt = 1;
    this->disappearCnt = 0;
    this->trackedCnt = 0;
    this->status = INSPECTING;
    this->tracker = tracker;
    switch (status) {
        case INSPECTING:
            if (appearCnt >= tracker->appear_thr) {
                this->status = TRACKING;
                this->id = tracker->object_index++;
            } else {
                this->status = INSPECTING;
            }
            break;
        default:
            break;
    }
}

void TrackedObj::disappear() {
    this->obj = this->prev_obj;
    this->obj_id = -1;
    this->disappearCnt++;
    if (disappearCnt >= tracker->disappear_thr) {
        status = LOSTING;
    }
}

void TrackedObj::appear(const autoware_msgs::DetectedObject &obj, int obj_id) {
    this->obj = obj;
    this->obj_id = obj_id;
    this->appearCnt++;
    this->disappearCnt = 0;
    switch (status) {
        case INSPECTING:
            if (appearCnt >= tracker->appear_thr) {
                this->status = TRACKING;
                this->id = tracker->object_index++;
            } else {
                this->status = INSPECTING;
            }
            break;
        default:
            break;
    }
}

void TrackedObj::release() {
}

void TrackedObj::smooth_update() {
}
