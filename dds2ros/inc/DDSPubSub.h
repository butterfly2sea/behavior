#ifndef DDSPUBSUB_ZYZN_COMMU_H
#define DDSPUBSUB_ZYZN_COMMU_H

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include <boost/signals2.hpp>

/*
* DDS发布订阅类，用于实际DDS话题的信息的发布及数据获取
*/
namespace zyzn
{
    
    namespace commu
    {
        using namespace eprosima::fastdds::dds;
        class CDDSPubSub : public DataReaderListener{
            public:
            typedef void (f_topicInfo_t) (CDDSPubSub *pubSub);
            typedef boost::signals2::signal<f_topicInfo_t> sig_topic_t;

            enum EQosLvl{
				Reliable = 0,
				BestEffort = 9,
                QosCount=2
            };

            sig_topic_t sig_topic;
            CDDSPubSub(const std::string & topicName,TypeSupport* type,DomainParticipant* dmp,Subscriber* sub,Publisher* pub,EQosLvl qos=BestEffort);
            ~CDDSPubSub() override;
            void on_data_available(DataReader* reader) override;
            void on_subscription_matched(
                DataReader*,
                const SubscriptionMatchedStatus& info) override;
			//释放绑定的资源
            void releaseRes(TypeSupport* type,DomainParticipant* dmp,Subscriber* sub,Publisher* pub);
            inline Topic* getTopic(){
                return m_topic;
            }
            inline void* getRcvData(){
                return m_rcvData;
            }
            inline void* getSendData(){
                return m_sendData;
            }
            inline void sendData(){

                m_writer->write(m_sendData);
            }


            private:
            DataReader* m_reader;                 //订阅话题reader对象
            DataWriter* m_writer;                 //发布话题writer对象
            Topic* m_topic;                       //话题信息
            DataWriterListener* m_listenerWrite;
            void* m_rcvData;                      //存放接收的数据
            void* m_sendData;                     //存放要发送的数据
            

        };
    } // namespace commu
    
} // namespace zyzn


#endif