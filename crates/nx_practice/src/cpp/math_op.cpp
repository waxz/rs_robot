#include "math_op.h"

#include <stdio.h>
int add_int(int a, int b){
    printf("[cpp add_int]: input: a = %i, b = %i\n",a,b);
    return a+b;
}

int add_func(int a, int b,ADD_FUNC f
){
    printf("[cpp add_func]: input: a = %i, b = %i\n",a,b);
    return f(a,b);
}
void print_msg(const char* msg){
    printf("[cpp print_msg]: %s \n",msg);
}
void* get_func(){
    return (void*)add_int;
}

