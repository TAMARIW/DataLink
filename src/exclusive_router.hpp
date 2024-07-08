/**
 * The exclusive router will work just like the standard router but adds the 
 * ability to exclude given topics from being forwarded from one gateway to another.\
 * @author Christopher Steffen
*/

#include "gateway/router.h"


class ExclusiveRouter : public RODOS::Router {
private:

    /// @brief The max number of topics that can be excluded.
    static constexpr uint16_t TOPIC_LIST_SIZE = 1000;

    /// @brief List containing the topics to not be fowarded.
    uint32_t topicList_[TOPIC_LIST_SIZE];
    uint16_t topicListLength_ = 0;


public:

    ExclusiveRouter(bool forwardTopicReports_ = false, RODOS::Gateway* gateway1=0, RODOS::Gateway* gateway2=0, RODOS::Gateway* gateway3=0, RODOS::Gateway* gateway4=0);

    /**
     * @brief Add the given topic to the exclusivity list. This topic will not be forwarded to another gateway.
     * @param topicID The topicID to not forward.
     * @returns true if topic added to list, false otherwise.
    */
    bool addTopicToExclude(uint32_t topicID);

    /**
     * @brief Add the given topic to the exclusivity list. This topic will not be forwarded to another gateway.
     * @param topic The topic to not forward.
     * @returns true if topic added to list, false otherwise.
    */
    template <class TYPE>
    bool addTopicToExclude(RODOS::Topic<TYPE>& topic) {return addTopicToExclude(topic->topicId);}

    /**
     * @brief resets the topic list. All topics previously added to be excluded will not be excluded anymore.
    */
    void resetTopics(); 


    bool shouldRouteThisMsg(RODOS::NetworkMessage &msg, uint32_t linkid) override;


};