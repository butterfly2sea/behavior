
#include <fastdds/dds/log/Log.hpp>
#ifdef __linux
#include <log/Logger.hpp>
#endif
#include"DDSPubSub.h"
namespace zyzn
{
    namespace commu
    {
        CDDSPubSub::CDDSPubSub(const std::string & topicName,TypeSupport* type,DomainParticipant* dmp,Subscriber* sub,Publisher* pub,EQosLvl qos):m_reader(nullptr)
        ,m_writer(nullptr)
        ,m_topic(nullptr)
        ,m_listenerWrite(nullptr)
        ,m_rcvData(nullptr)
        ,m_sendData(nullptr)
        {
            if(dmp){
                char profile[128]={0};
                sprintf(profile,"level%d_topic_profile",qos);
                m_topic = dmp->create_topic_with_profile(topicName,type->get_type_name(),profile);   
                if(!m_topic){
                    #ifdef __linux
                    txtLog().error(THISMODULE "create topic failed");
                    #endif
                }     
                
            } 

			//  
            if(sub && m_topic){
         
                char profile[128]={0};
                sprintf(profile,"level%d_datareader_profile",qos);
                m_reader= sub->create_datareader_with_profile(m_topic,profile,this);

            }

			//
            if(pub && m_topic){

                char profile[128]={0};
                sprintf(profile,"level%d_datawriter_profile",qos);
                m_listenerWrite = new DataWriterListener();
                m_writer = pub->create_datawriter_with_profile(m_topic,profile,m_listenerWrite);
            }

			//
            if(m_reader && type)
                m_rcvData = type->create_data();
			//
            if(m_writer && type)
                m_sendData = type->create_data();
            
        }
        CDDSPubSub::~CDDSPubSub(){
            if(m_listenerWrite){
                delete m_listenerWrite;
                m_listenerWrite = nullptr;
            }
        }
        void CDDSPubSub::on_data_available(DataReader* reader){
            SampleInfo info;
            if (reader->take_next_sample(m_rcvData, &info) == RETCODE_OK)
            {               
                if (info.valid_data)
                {
					//
                    sig_topic(this);
                }
            }
        }
        void CDDSPubSub::on_subscription_matched(
                DataReader*,
                const SubscriptionMatchedStatus& info) 
        {
            std::string msg;
            if (info.current_count_change == 1)
            {
                msg = "Subscriber matched. topicName: ";
               
            }
            else if (info.current_count_change == -1)
            {
                msg = "Subscriber unmatched. topicName: ";
              
            }
            else
            {
                msg =  " is not a valid value for SubscriptionMatchedStatus current count change. topicName:";
               
            }
            msg += m_topic->get_name();

            #ifdef __linux
            txtLog().info(THISMODULE "%s",msg.c_str());
            #endif
        }
        void CDDSPubSub::releaseRes(TypeSupport* type,DomainParticipant* dmp,Subscriber* sub,Publisher* pub){
            if(sub && m_reader){
                sub->delete_datareader(m_reader);
                m_reader = nullptr;
            }
           
            if(pub && m_writer){
                pub->delete_datawriter(m_writer);
                m_writer = nullptr;
            }
            if(dmp && m_topic){
                dmp->delete_topic(m_topic);
                m_topic = nullptr;
            }
            if(type && m_rcvData){
                type->delete_data(m_rcvData);
                m_rcvData = nullptr;
            }
            if(type && m_sendData){
                type->delete_data(m_sendData);
                m_sendData = nullptr;
            }
        }
    } // namespace commu
    
} // namespace zyzn
