#ifndef _IPTESTER_H
#define _IPTESTER_H

/**
 * @brief Enumeration of the iptesting states. Seems enums must be in a header due to arduino bugs. I'll live.
 * 
 */
enum testing_state {
    idle,
    initialization,
    test,
    result,
    test_abort,
    test_complete,
    error
};

enum rgb_state {
    off,
    red,
    green,
    blue,
    orange,
    purple,
    yellow,
    white
};


#endif
