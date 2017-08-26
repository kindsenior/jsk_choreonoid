
#include <cnoid/PyUtil>
#include <cnoid/EigenTypes>
#include <cnoid/Body>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/range/value_type.hpp>

namespace python = boost::python;
using namespace cnoid;

namespace {

    python::object numpy;
    python::object numpy_ndarray;
    python::object numpy_array;

    template<typename VectorType>
    struct Vector_to_pylist_converter {
        static PyObject* convert(const VectorType& v){
            python::list elements;
            for(int i = 0; i < v.size(); ++i){
                elements.append(v[i]);
            }
            return python::incref(elements.ptr());
        }
    };

    // template<typename VectorType> // TODO
    // struct Vector_to_ndarray_converter {
    //     static PyObject* convert(const VectorType& v){
    //         python::list elements;
    //         for(int i = 0; i < v.size(); ++i){
    //             elements.append(v[i]);
    //         }
    //         python::object array = numpy_array(elements);
    //         return python::incref(array.ptr());
    //     }
    // };

    template<typename VectorType>
    struct ndarray_to_VectorX_converter {
        ndarray_to_VectorX_converter(){
            python::converter::registry::push_back(
                                                   &ndarray_to_VectorX_converter::convertible,
                                                   &ndarray_to_VectorX_converter::construct,
                                                   python::type_id<VectorType>());
        }
        static void* convertible(PyObject* pyobj){
            if(PyObject_IsInstance(pyobj, numpy_ndarray.ptr()) == 1){
                return pyobj;
            }
            return 0;
        }
        static void construct(PyObject* pyobj, python::converter::rvalue_from_python_stage1_data* data){
            VectorType* pv = new(reinterpret_cast<python::converter::rvalue_from_python_storage<VectorType>*>
                                 (data)->storage.bytes) VectorType();
            int dim = PySequence_Size(pyobj);
            (*pv).resize(dim);
            for(python::ssize_t i = 0; i < dim; ++i) {
                (*pv)[i] = python::extract<typename VectorType::Scalar>(PySequence_GetItem(pyobj, i));
            }
            data->convertible = pv;
        }
    };

    template<typename VectorType>
    struct pylist_to_VectorX_converter {
        pylist_to_VectorX_converter(){
            python::converter::registry::push_back(
                                                   &pylist_to_VectorX_converter::convertible,
                                                   &pylist_to_VectorX_converter::construct,
                                                   python::type_id<VectorType>());
        }
        static void* convertible(PyObject* pyobj){
            if(PySequence_Check(pyobj)){
                return pyobj;
            }
            return 0;
        }
        static void construct(PyObject* pyobj, python::converter::rvalue_from_python_stage1_data* data){
            VectorType* pv = new(reinterpret_cast<python::converter::rvalue_from_python_storage<VectorType>*>
                                 (data)->storage.bytes) VectorType();
            int dim = PySequence_Size(pyobj);
            (*pv).resize(dim);
            for(python::ssize_t i = 0; i < dim; ++i) {
                (*pv)[i] = python::extract<typename VectorType::Scalar>(PySequence_GetItem(pyobj, i));
            }
            data->convertible = pv;
        }
    };
}

namespace cnoid {
    void exportPyTypesPlugin()
    {
        numpy = python::import("numpy");
        numpy_ndarray = numpy.attr("ndarray");
        numpy_array = numpy.attr("array");

        // vectorX
        python::to_python_converter<VectorX,
                                    Vector_to_pylist_converter<VectorX> >();
        ndarray_to_VectorX_converter<VectorX>();
        pylist_to_VectorX_converter<VectorX>();

        // std::vector<Link>
        python::to_python_converter<std::vector<Link*>,
                                    Vector_to_pylist_converter<std::vector<Link*>> >();
    }
}
