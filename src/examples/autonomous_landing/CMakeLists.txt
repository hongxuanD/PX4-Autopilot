px4_add_module(
    MODULE examples__autonomous_landing
    MAIN autonomous_landing
    STACK_MAIN 2000
    SRCS
        autonomous_landing.c
    DEPENDS
    )
