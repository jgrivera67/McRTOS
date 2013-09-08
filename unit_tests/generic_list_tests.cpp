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
    struct list_node list_anchor;
    struct list_node list_elem1;
    struct list_node list_elem2;

    void
    setup()
    {
        LIST_NODE_INIT(&list_anchor);
        LIST_NODE_INIT(&list_elem1);
        LIST_NODE_INIT(&list_elem2);
    }

    void teardown()
    {
    }
};


TEST(generic_list, add_head_remove)
{
    CHECK(LIST_EMPTY(&list_anchor));
    CHECK(list_node_unlinked(&list_elem1));

    list_add_head_elem(&list_anchor, &list_elem1);

    CHECK(!LIST_EMPTY(&list_anchor));
    CHECK(!list_node_unlinked(&list_elem1));
    CHECK(LIST_GET_COUNT(&list_anchor) == 1);
    CHECK(LIST_GET_FIRST(&list_anchor) == &list_elem1);

    list_remove_elem(&list_elem1);

    CHECK(LIST_EMPTY(&list_anchor));
    CHECK(list_node_unlinked(&list_elem1));
}


TEST(generic_list, add_tail_add_tail_remove)
{
    struct list_node *elem_p;
    struct list_node *next_p;

    CHECK(LIST_EMPTY(&list_anchor));
    CHECK(list_node_unlinked(&list_elem1));
    CHECK(list_node_unlinked(&list_elem2));

    list_add_tail_elem(&list_anchor, &list_elem1);
    list_add_tail_elem(&list_anchor, &list_elem2);

    CHECK(!LIST_EMPTY(&list_anchor));
    CHECK(!list_node_unlinked(&list_elem1));
    CHECK(!list_node_unlinked(&list_elem2));
    CHECK(LIST_GET_COUNT(&list_anchor) == 2);
    CHECK(LIST_GET_FIRST(&list_anchor) == &list_elem1);
    CHECK(LIST_GET_LAST(&list_anchor) == &list_elem2);

    LIST_FOR_EACH_NODE_REMOVING(elem_p, next_p, &list_anchor)
    {
        list_remove_elem(elem_p);
    }

    CHECK(LIST_EMPTY(&list_anchor));
    CHECK(list_node_unlinked(&list_elem1));
    CHECK(list_node_unlinked(&list_elem2));
}
