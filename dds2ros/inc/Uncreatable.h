#ifndef UNCREATABLE_ZYZN_H
#define UNCREATABLE_ZYZN_H 

namespace zyzn
{
    class CUncreatable{
        
        CUncreatable(const CUncreatable& ref);
        CUncreatable& operator =(const CUncreatable& ref);
        protected:
        CUncreatable(){}
        public:
        virtual ~CUncreatable(){};

    };
    
} // namespace zyzn


#endif