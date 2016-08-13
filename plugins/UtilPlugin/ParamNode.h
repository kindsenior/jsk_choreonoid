/**
   @author Kunio Kojima
*/
#pragma once

#include <cnoid/Archive>

namespace cnoid{

class ParamNode
{
protected:
    std::string saveName_;
    std::string archiveName_;

public:
    ParamNode()
    {
        saveName_ = "defaultSaveName";
        archiveName_ = "defaultArchiveName";
    }

    // virtual void storeState(Archive& archive)=0;
    // virtual void restoreState(const Archive& archive)=0;
    virtual void storeState(Archive& archive){};
    virtual void restoreState(const Archive& archive){};

    virtual std::string getParamString(){std::cout << "Prease implement getParamString() of each kind of ParamNode()" << std::endl;};
    virtual void addParamNode(ParamNode* paramNode){};

    void setSaveName(const std::string& name){saveName_ = name;}
    std::string saveName(){return saveName_;}
    void setArchiveName(const std::string& name){archiveName_ = name;}
    std::string archiveName(){return archiveName_;}
};

class ParamMap : public ParamNode,
                 public std::map<std::string,ParamNode*>
{
public:
    ParamMap()
        : ParamNode(),
          std::map<std::string,ParamNode*>()
    {
    }

    virtual void storeState(Archive& archive)
    {
        Mapping& mapping = *archive.createMapping(archiveName_);
        for(std::map<std::string, ParamNode*>::iterator iter = this->begin(); iter != this->end(); ++iter){
            (*iter).second->storeState((Archive&) mapping);
        }
    }

    virtual void restoreState(const Archive& archive)
    {
        std::cout << "ParamMap::restoreState()" << std::endl;
        Mapping& mapping = *archive.findMapping(archiveName_);
        if(mapping.isValid() && !mapping.empty()){
            for(std::map<std::string, ParamNode*>::iterator iter = this->begin(); iter != this->end(); ++iter){
                (*iter).second->restoreState((Archive&) mapping);
            }
        }
    }

    virtual std::string getParamString()
    {
        std::stringstream ss;
        ss << "_" << archiveName_;
        for(std::map<std::string, ParamNode*>::iterator iter = this->begin(); iter != this->end(); ++iter){
            ss << (*iter).second->getParamString();
        }
        return ss.str();
    }

    virtual void addParamNode(ParamNode* paramNode)
    {
        std::stringstream ss;
        ss << size() << paramNode->archiveName();
        (*this)[ss.str()] = paramNode;
    };
};

}
