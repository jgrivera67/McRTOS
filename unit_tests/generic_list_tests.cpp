///
/// @file generic_list.cpp
///
/// McRTOS unit tests.
/// 
/// @author: German Rivera
///

#include <stdio.h>
#include <stdarg.h>
#include "CppUTest/TestHarness.h"

extern "C"
{
#   include "generic_list.h"
}


TEST_GROUP(generic_list)
{
    struct glist_node list_anchor;
    struct glist_node list_elem1;
    struct glist_node list_elem2;

    void
    setup()
    {
        GLIST_NODE_INIT(&list_anchor);
        GLIST_NODE_INIT(&list_elem1);
        GLIST_NODE_INIT(&list_elem2);
    }

    void teardown()
    {
    }
};


TEST(generic_list, add_head_remove)
{
    CHECK(GLIST_IS_EMPTY(&list_anchor));
    CHECK(glist_node_is_unlinked(&list_elem1));

    glist_add_head_elem(&list_anchor, &list_elem1);

    CHECK(!GLIST_IS_EMPTY(&list_anchor));
    CHECK(!glist_node_is_unlinked(&list_elem1));
    CHECK(GLIST_GET_NODE_COUNT(&list_anchor) == 1);
    CHECK(GLIST_GET_FIRST(&list_anchor) == &list_elem1);

    glist_remove_elem(&list_elem1);

    CHECK(GLIST_IS_EMPTY(&list_anchor));
    CHECK(glist_node_is_unlinked(&list_elem1));
}


TEST(generic_list, add_tail_add_tail_remove)
{
    struct glist_node *elem_p;
    struct glist_node *next_p;

    CHECK(GLIST_IS_EMPTY(&list_anchor));
    CHECK(glist_node_is_unlinked(&list_elem1));
    CHECK(glist_node_is_unlinked(&list_elem2));

    glist_add_tail_elem(&list_anchor, &list_elem1);
    glist_add_tail_elem(&list_anchor, &list_elem2);

    CHECK(!GLIST_IS_EMPTY(&list_anchor));
    CHECK(!glist_node_is_unlinked(&list_elem1));
    CHECK(!glist_node_is_unlinked(&list_elem2));
    CHECK(GLIST_GET_NODE_COUNT(&list_anchor) == 2);
    CHECK(GLIST_GET_FIRST(&list_anchor) == &list_elem1);
    CHECK(GLIST_GET_LAST(&list_anchor) == &list_elem2);

    GLIST_FOR_EACH_NODE_REMOVING(elem_p, next_p, &list_anchor)
    {
        glist_remove_elem(elem_p);
    }

    CHECK(GLIST_IS_EMPTY(&list_anchor));
    CHECK(glist_node_is_unlinked(&list_elem1));
    CHECK(glist_node_is_unlinked(&list_elem2));
}
