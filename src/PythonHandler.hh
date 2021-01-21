#include<iostream>
#include <dlfcn.h>
#include <link.h>
#include <string>
#include <vector>


std::vector<std::string> loaded_libraries;

int dl_list_callback(struct dl_phdr_info *info, size_t, void *)
{
    loaded_libraries.push_back(info->dlpi_name);
    return 0;
}



using namespace std;
class PythonHandler
{


public:
        void* pFunc;
        void* pFunc3;
        void* Model;
        void* pHandle;


        typedef void (*void_func_t)(void);
        typedef void* (*void_func_t2)(char*);
        typedef void* (*void_func_t3)(void*);
        typedef void* (*void_func_t4)(int);
        typedef void* (*void_func_t5)(void*,char*);
        typedef void* (*void_func_t6)(char*,char*);
        typedef void (*void_func_t7)(void*,int,void*);
        typedef void* (*void_func_t8)(void*,void*);
        typedef double (*void_func_t9)(void*);
        typedef void* (*void_func_t10)(double);
        typedef void* (*void_func_t11)(void*,int);
        typedef void (*void_func_t12)(void*);


        void_func_t4 MyPyTuple_New;
        void_func_t MyPy_Finalize;
        void_func_t6 MyPy_BuildValue;
        void_func_t7 MyPyTuple_SetItem;
        void_func_t8 MyPyObject_CallObject;
        void_func_t9 MyPyFloat_AsDouble;

        void_func_t10 MyPyFloat_FromDouble;
        void_func_t7 MyPyList_SetItem;
        void_func_t4 MyPyList_New;
        void_func_t11 MyPyList_GetItem;
        void_func_t12 MyPy_DECREF;
        void_func_t4 MyPyLong_FromLong;



PythonHandler()
{



    using std::cerr;
     pHandle = dlopen("libpython3.6m.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!pHandle ) {
        cerr << "Cannot open library: " << dlerror() << '\n';
    }

    void_func_t MyPy_Initialize = (void_func_t)dlsym(pHandle, "Py_Initialize");
    void_func_t MyPyErr_Print= (void_func_t)dlsym(pHandle, "PyErr_Print");
     MyPy_Finalize = (void_func_t)dlsym(pHandle, "Py_Finalize");
    void_func_t2 MyPy_SimpleString = (void_func_t2)dlsym(pHandle, "PyRun_SimpleString");
    void_func_t2 MyPy_FromString= (void_func_t2)dlsym(pHandle, "PyUnicode_FromString");
    void_func_t3 MyPy_Import= (void_func_t3)dlsym(pHandle, "PyImport_Import");
    void_func_t3 MyPyModule_GetDict= (void_func_t3)dlsym(pHandle, "PyModule_GetDict");
    void_func_t5 MyPyDict_GetItemString=(void_func_t5)dlsym(pHandle, "PyDict_GetItemString");
    MyPy_BuildValue=(void_func_t6)dlsym(pHandle, "Py_BuildValue");
    MyPyTuple_New=(void_func_t4)dlsym(pHandle, "PyTuple_New");
    MyPyTuple_SetItem=(void_func_t7)dlsym(pHandle, "PyTuple_SetItem");
    MyPyObject_CallObject=(void_func_t8)dlsym(pHandle, "PyObject_CallObject");
    MyPyFloat_AsDouble=(void_func_t9)dlsym(pHandle, "PyFloat_AsDouble");

    MyPyFloat_FromDouble=(void_func_t10)dlsym(pHandle, "PyFloat_FromDouble");
    MyPyList_SetItem=(void_func_t7)dlsym(pHandle, "PyList_SetItem");

    if(!MyPyList_SetItem) std::cout<<"NULL Pointer"<<std::endl;


    MyPyList_New=(void_func_t4)dlsym(pHandle, "PyList_New");
    MyPyList_GetItem=(void_func_t11)dlsym(pHandle, "PyList_GetItem");
    MyPyLong_FromLong=(void_func_t4)dlsym(pHandle, "PyLong_FromLong");
    if(!MyPyLong_FromLong) std::cout<<"NULL Pointer"<<std::endl;


    MyPy_Initialize();
    MyPy_SimpleString("import sys");
    MyPy_SimpleString("import os");
    MyPy_SimpleString("sys.path.append(os.getcwd())");
    void* ModuleString=MyPy_FromString("action_gen");
    void* Module = MyPy_Import(ModuleString);


    std::cout<<"____________________________________________________________________"<<std::endl;
    void* pDict = MyPyModule_GetDict(Module);
    void* pFunc2 = MyPyDict_GetItemString(pDict, "LoadModel");
    pFunc = MyPyDict_GetItemString(pDict, "act");
    pFunc3 = MyPyDict_GetItemString(pDict, "act2");


    void* prm2 = MyPy_BuildValue("d", "0");
    void* pArgs2 = MyPyTuple_New(1);
    MyPyTuple_SetItem(pArgs2, 0, prm2);

    Model = MyPyObject_CallObject(pFunc2, pArgs2);


//    void* pArgs = MyPyTuple_New(2);
//    void* List= MyPyList_New(3);

//    void* prm5 = MyPyFloat_FromDouble(0);
//    MyPyList_SetItem(List, 0, prm5);
//    void* prm6 = MyPyFloat_FromDouble(0);
//    MyPyList_SetItem(List, 1, prm6);
//    void* prm7 = MyPyFloat_FromDouble(0);
//    MyPyList_SetItem(List, 2, prm7);

////   for (int i = 0; i < 3; i++)
////   {
////       void* prm = MyPyFloat_FromDouble(0);
////       MyPyList_SetItem(List, i, prm);

////   }
//   std::cout<<"here is reached"<<std::endl;

//   MyPyTuple_SetItem(pArgs, 0, List);
//   MyPyTuple_SetItem(pArgs, 1, Model);

//   if(!pFunc) std::cout<<"NULL Pointer"<<std::endl;


//   void* ret = MyPyObject_CallObject(pFunc, pArgs);
//   if (!ret) {
//        MyPyErr_Print();
//       printf("Error return\n");
//   }


//   cout<<MyPyFloat_AsDouble(MyPyList_GetItem(ret,0))<<endl;
//   cout<<MyPyFloat_AsDouble(MyPyList_GetItem(ret,1))<<endl;



    cout<<"____________________________________________________________________"<<endl;
    cout<<"Importing is finished"<<endl;

}

void destructor(){
    MyPy_Finalize();

    loaded_libraries.clear();
    dl_iterate_phdr(dl_list_callback, NULL);

    std::size_t start_idx(loaded_libraries.size());

    for(std::size_t i(loaded_libraries.size()-1) ; i >= start_idx ; --i) {
            void* pHandle = dlopen(loaded_libraries[i].c_str(),
    #ifdef WIN32
                                   RTLD_NOW // no support for RTLD_NOLOAD
    #else
                                   RTLD_NOW|RTLD_NOLOAD
    #endif /* WIN32 */
                        );
            if (pHandle) {
                const unsigned int Nmax(50); // Avoid getting stuck in an infinite loop
                for (unsigned int j(0) ; j < Nmax && !dlclose(pHandle) ; ++j);
            }
        }

    dlclose(pHandle);
    pHandle = NULL;

}

template<class state_type,class input_type>
std::vector<input_type> get_controller2(std::vector<state_type>& states,const unsigned int state_dim){

    //PyObject* pArgs,List,prm,ret;

     void* pArgs = MyPyTuple_New(3);
     if(!pArgs) std::cout<<"NULL Pointer"<<std::endl;
     void* x_list= MyPyList_New(states.size());

     for (int k = 0; k < states.size(); ++k) {
     void* x= MyPyList_New(state_dim);

    for (int i = 0; i < state_dim; i++)
    {
        void* prm = MyPyFloat_FromDouble(states[k][i]);
        MyPyList_SetItem(x, i, prm);

    }
    MyPyList_SetItem(x_list, k, x);
     }

    MyPyTuple_SetItem(pArgs, 0, x_list);
    MyPyTuple_SetItem(pArgs, 1, Model);
    MyPyTuple_SetItem(pArgs, 2, MyPyLong_FromLong(states.size()));




    void* ret = MyPyObject_CallObject(pFunc, pArgs);
    if (!ret) {
//        MyPyErr_Print();
        printf("Error return\n");
    }


    //string result = PyBytes_AsString(ret);
    //cout<<PyFloat_AsDouble(PyList_GetItem(ret,0))<<endl;
    //cout<<PyFloat_AsDouble(PyList_GetItem(ret,1))<<endl;

     input_type input;
     std::vector<input_type> output;
//     output[0]=MyPyFloat_AsDouble(MyPyList_GetItem(ret,0));
//     output[1]=MyPyFloat_AsDouble(MyPyList_GetItem(ret,1));
     for (int i = 0; i < states.size(); ++i) {
         input_type input;
          input[0]=MyPyFloat_AsDouble(MyPyList_GetItem(MyPyList_GetItem(ret,i),0));
          input[1]=MyPyFloat_AsDouble(MyPyList_GetItem(MyPyList_GetItem(ret,i),1));
         output.push_back(input);
     }


     return output;

}






template<class state_type,class input_type>
input_type get_controller(state_type& x,const unsigned int state_dim){
    
    //PyObject* pArgs,List,prm,ret;

     void* pArgs = MyPyTuple_New(2);
     if(!pArgs) std::cout<<"NULL Pointer"<<std::endl;
     void* List= MyPyList_New(state_dim);


    for (int i = 0; i < state_dim; i++)
    {
        void* prm = MyPyFloat_FromDouble(x[i]);
        MyPyList_SetItem(List, i, prm);
    
    }

    MyPyTuple_SetItem(pArgs, 0, List);
    MyPyTuple_SetItem(pArgs, 1, Model);
        


    void* ret = MyPyObject_CallObject(pFunc3, pArgs);
    if (!ret) {
//        MyPyErr_Print();
        printf("Error return\n");
    }

    
    //string result = PyBytes_AsString(ret);
    //cout<<PyFloat_AsDouble(PyList_GetItem(ret,0))<<endl;
    //cout<<PyFloat_AsDouble(PyList_GetItem(ret,1))<<endl;
     input_type output;
     output[0]=MyPyFloat_AsDouble(MyPyList_GetItem(ret,0));
     output[1]=MyPyFloat_AsDouble(MyPyList_GetItem(ret,1));


     return output;
    
}



~PythonHandler()
{
}
};
