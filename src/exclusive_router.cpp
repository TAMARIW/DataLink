/**
 * The exclusive router will work just like the standard router but adds the 
 * ability to exclude given topics from being forwarded from one gateway to another.\
 * @author Christopher Steffen
*/

#include "gateway/router.h"

#include "exclusive_router.hpp"


ExclusiveRouter::ExclusiveRouter(bool forwardTopicReports_, RODOS::Gateway* gateway1, RODOS::Gateway* gateway2, RODOS::Gateway* gateway3, RODOS::Gateway* gateway4) :
    Router(forwardTopicReports_, gateway1, gateway2, gateway3, gateway4)
{}

bool ExclusiveRouter::addTopicToExclude(uint32_t topicID) {

    if (topicListLength_ >= TOPIC_LIST_SIZE) 
        return false;

    topicList_[topicListLength_] = topicID;
    topicListLength_++;

    return true;

}

void ExclusiveRouter::resetTopics() {
    topicListLength_ = 0;
}

bool ExclusiveRouter::shouldRouteThisMsg(RODOS::NetworkMessage &msg, uint32_t linkid) {

    if (msg.get_maxStepsToForward() <= 0)               return false;
    if (msg.get_topicId() == 0 && !forwardTopicReports) return false;

    for (uint16_t i = 0; i < topicListLength_; i++) {
        if (msg.get_topicId() == topicList_[i])
            return false;
    }
    
    return true;

}
