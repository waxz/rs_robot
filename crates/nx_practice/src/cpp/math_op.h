#ifndef MATH_OP_H
#define MATH_OP_H


#ifdef __cplusplus
extern "C" {
#endif


    typedef int(*ADD_FUNC)(int, int ) ;

    /// add two int\n
    /// \param a\n
    /// \param b\n
    /// \return c = a + b
    int add_int(int a, int b);

    /// add_func with function pointer
    /// \param a
    /// \param b
    /// \param f : function pointer
    /// \return
    int add_func(int a, int b , ADD_FUNC f );

    void* get_func();

    void print_msg(const char* msg);


#ifdef __cplusplus
}
#endif
#endif
