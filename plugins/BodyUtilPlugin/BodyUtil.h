

#include <vector>
#include <iostream>
#include <cnoid/Body>
#include <cnoid/EigenTypes>


namespace cnoid {

    std::vector<Link*> links(const Body& body)
    {
        size_t num = body.numLinks();
        std::vector<Link*> ret(num);
        for (size_t i = 0; i < num; i++)
            ret[i] = body.link(i);
        return ret;
    }

    std::vector<Link*> jointList(const Body& body)
    {
        size_t num = body.numJoints();
        std::vector<Link*> ret(num);
        for (size_t i = 0; i < num; i++)
            ret[i] = body.joint(i);
        return ret;
    }

    VectorX angleVector(const Body& body, const VectorX& angles)
    {
        size_t num = body.numJoints();
        VectorX ret(num);
        if ( num == angles.size() ) {
            for (size_t i = 0; i < num; i++)
                body.joint(i)->q() = angles[i];
        } else {
            std::cerr << "TypeError: length of angles do not agree with self.numJoints" << std::endl;
        }
        for (size_t i = 0; i < num; i++)
            ret[i] = body.joint(i)->q();
        return ret;
    }
}
