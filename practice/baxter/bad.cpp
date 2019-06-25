#include<stdio.h>

int main (void)
{
    printf("Two plus two is %f\n",4);
    #if 1
        printf("hehe\n");
    #endif
    #if 0
        printf("true\n");
    #else
        printf("false\n");
    #endif
    return 0;

}
/* zhe shi zhu shi */
/* haha
* hehe
*/
// zhe ta ma ye shi zhu shi

