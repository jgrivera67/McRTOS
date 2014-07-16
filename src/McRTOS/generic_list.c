/**
 * @file generic_list.c
 *
 * Generic linked lists.
 *
 * @author: German Rivera
 */

#include "generic_list.h"
#include "failure_data_capture.h"

/**
 * Tell if a list list node is not linked to any other list node.
 *
 * @param list_node_p	pointer to the list node.
 *
 * @return true, if the list node is lined to itself (not linked to another node)
 * @return false, otherwise
 */
bool
glist_node_is_unlinked(const struct glist_node *list_node_p)
{
    if (list_node_p->ln_next_p == list_node_p) {
            DBG_ASSERT(
                list_node_p->ln_prev_p == list_node_p,
                list_node_p->ln_prev_p,
                list_node_p);

            DBG_ASSERT(
                 list_node_p->ln_node_count == 0,
                 list_node_p->ln_node_count,
                 list_node_p);

            return true;
    } else {
            DBG_ASSERT(
                list_node_p->ln_prev_p != list_node_p,
                list_node_p->ln_prev_p,
                list_node_p);

            DBG_ASSERT(
                list_node_p->ln_node_count != 0,
                list_node_p->ln_node_count,
                list_node_p);

            return false;
    }

} /* glist_node_unlinked */


/**
 * Adds an element at the beginning of a list
 * in the list
 *
 * @param list_p	        pointer to a list's anchor list node
 * @param elem_p		pointer to the element to be inserted
 *
 * @return none
 *
 * @pre The caller is expected to serialize access to the list from
 *      multiple threads
 */
void
glist_add_head_elem(struct glist_node *list_p, struct glist_node *elem_p)
{
    FDC_ASSERT_GLIST_ANCHOR_INVARIANTS(list_p);
    FDC_ASSERT_GLIST_ELEM_INVARIANTS(elem_p);

    /*
     * elem_p is not already in a list:
     */
    FDC_ASSERT(
        elem_p->ln_anchor_p == NULL,
        elem_p->ln_anchor_p,
        elem_p);

    elem_p->ln_next_p = list_p->ln_next_p;
    elem_p->ln_prev_p = list_p;
    list_p->ln_next_p->ln_prev_p = elem_p;
    list_p->ln_next_p = elem_p;

#ifdef _RELIABILITY_CHECKS_

    list_p->ln_node_count ++;
    elem_p->ln_anchor_p = list_p;

#endif /* _RELIABILITY_CHECKS_ */

} /* glist_add_head_elem */


/**
 * Adds an element at the end of a list or before another element
 * in the list
 *
 * @param list_p	        pointer to the list's anchor node
 * @param elem_p		pointer to the element to be inserted
 *
 * @return none
 *
 * @pre The caller is expected to serialize access to the
 *	list from multiple threads
 */
void
glist_add_tail_elem(struct glist_node *list_p, struct glist_node *elem_p)
{
    FDC_ASSERT_GLIST_ANCHOR_INVARIANTS(list_p);
    FDC_ASSERT_GLIST_ELEM_INVARIANTS(elem_p);

    /*
     * elem_p is not already in a list:
     */
    FDC_ASSERT(elem_p->ln_anchor_p == NULL,
                 elem_p->ln_anchor_p,
                 elem_p);

    elem_p->ln_next_p = list_p;
    elem_p->ln_prev_p = list_p->ln_prev_p;
    list_p->ln_prev_p->ln_next_p = elem_p;
    list_p->ln_prev_p = elem_p;

#ifdef _RELIABILITY_CHECKS_

    list_p->ln_node_count ++;
    elem_p->ln_anchor_p = list_p;

#endif /* _RELIABILITY_CHECKS_ */

} /* glist_add_tail_elem */


/**
 * Remove an element from the list it is currently in and resets the links of
 * the element being removed, to point to itself.
 *
 * @param elem_p	pointer to the element
 *
 * @return none
 *
 * @pre The caller is expected to serialize access to the
 *	list from multiple threads
 */
void
glist_remove_elem(struct glist_node *elem_p)
{
#ifdef _RELIABILITY_CHECKS_
    struct glist_node *list_p = elem_p->ln_anchor_p;
#endif

    /*
     * elem_p is in a list:
     */
    FDC_ASSERT(list_p != NULL,
        list_p,
        elem_p);

    FDC_ASSERT_GLIST_ELEM_INVARIANTS(elem_p);

    elem_p->ln_prev_p->ln_next_p = elem_p->ln_next_p;
    elem_p->ln_next_p->ln_prev_p = elem_p->ln_prev_p;

#ifdef _RELIABILITY_CHECKS_

    elem_p->ln_anchor_p->ln_node_count --;
    elem_p->ln_anchor_p = NULL;

#endif /* _RELIABILITY_CHECKS_ */

    /*
     * Reset links of the removed node to point to itself:
     */
    elem_p->ln_next_p = elem_p;
    elem_p->ln_prev_p = elem_p;

    FDC_ASSERT_GLIST_ANCHOR_INVARIANTS(list_p);

} /* glist_remove_elem */


/**
 * Move the element nodes from the source list to the beginning of the
 * destination list. The source list becomes empty after this.
 *
 * @param src_list_p	pointer to the source list's anchor node
 * @param dest_list_p	pointer to the target list's anchor node
 *
 * @return none
 *
 * @pre The caller is expected to serialize access to the
 *	source and target lists from multiple threads
 */
void
glist_migrate_nodes(struct glist_node *src_list_p, struct glist_node *dest_list_p)
{
    FDC_ASSERT(src_list_p != dest_list_p, src_list_p, dest_list_p);

    FDC_ASSERT_GLIST_ANCHOR_INVARIANTS(src_list_p);
    FDC_ASSERT_GLIST_ANCHOR_INVARIANTS(dest_list_p);

#ifdef _RELIABILITY_CHECKS_
    if (src_list_p->ln_node_count == 0)
    {
        return;
    }
#endif /* _RELIABILITY_CHECKS_ */

    src_list_p->ln_prev_p->ln_next_p = dest_list_p->ln_next_p;
    src_list_p->ln_next_p->ln_prev_p = dest_list_p;
    dest_list_p->ln_next_p->ln_prev_p = src_list_p->ln_prev_p;
    dest_list_p->ln_next_p = src_list_p->ln_next_p;

    dest_list_p->ln_node_count += src_list_p->ln_node_count;

#ifdef _RELIABILITY_CHECKS_

    /*
     * Update the anchor node for the migrated nodes:
     */

    struct glist_node *elem_p;

    GLIST_FOR_EACH_NODE(elem_p, src_list_p) {
        FDC_ASSERT_GLIST_ELEM_INVARIANTS(elem_p);

        elem_p->ln_anchor_p = dest_list_p;
        src_list_p->ln_node_count --;
        if (src_list_p->ln_node_count == 0) {
            break;
        }
    }

#endif /* _RELIABILITY_CHECKS_ */

    FDC_ASSERT(src_list_p->ln_node_count == 0,
	   src_list_p->ln_node_count, src_list_p);

    FDC_ASSERT_GLIST_ANCHOR_INVARIANTS(dest_list_p);

    /*
     * Reset the links of src_list_p:
     */
    src_list_p->ln_next_p = src_list_p;
    src_list_p->ln_prev_p = src_list_p;

} /* glist_migrate_nodes */


#ifdef _RELIABILITY_CHECKS_

/**
 * Check correctness invariants of a list:
 * - If the list is empty, its ln_next_p and ln_prev_p pointers point to the list itself
 * - If the list is not empty:
 *   1. The list's ln_next_p and ln_prev_p pointers cannot point to the list itself.
 *   2. The list's ln_next_p and ln_prev_p pointers point to the same node if and only if
 *      the number of elements in the list is 1.
 */
void
glist_anchor_check_invariants(struct glist_node *list_p)
{
    FDC_ASSERT_VALID_RAM_POINTER(list_p, sizeof(void *));

    if (list_p->ln_node_count == 0) {
        FDC_ASSERT(
            list_p->ln_next_p == list_p,
            list_p->ln_next_p,
            list_p);

        FDC_ASSERT(
            list_p->ln_prev_p == list_p,
            list_p->ln_prev_p,
            list_p);
    } else {
        FDC_ASSERT(
            list_p->ln_next_p != list_p,
            list_p->ln_next_p,
            list_p);

        FDC_ASSERT(
            list_p->ln_prev_p != list_p,
            list_p->ln_prev_p,
            list_p);

        /*
         * list_p->ln_next_p == list_p->ln_prev_p <=> list_p->ln_node_count == 1
         */
        FDC_ASSERT(
            (list_p->ln_next_p == list_p->ln_prev_p &&
             list_p->ln_node_count == 1)
            ||
            (list_p->ln_next_p != list_p->ln_prev_p &&
             list_p->ln_node_count > 1),
            list_p->ln_node_count,
            list_p);

        FDC_ASSERT(
            list_p->ln_next_p->ln_anchor_p == list_p,
            list_p->ln_next_p->ln_anchor_p,
            list_p);

        FDC_ASSERT(
            list_p->ln_prev_p->ln_anchor_p == list_p,
            list_p->ln_prev_p->ln_anchor_p,
            list_p);
    }

} /* glist_anchor_check_invariants */


/**
 * Check correctness invariants of an element of a list:
 * - If the element does not belong to a list, its ln_next_p/ln_prev_p pointers point to
 *   itself.
 * - If the element belongs to a list:
 *   1. the element's ln_next_p/ln_prev_p pointer cannot point to the element itself
 *   2. The ln_next_p pointer of the element's predecessor in the list points to the
 *      element
 *   3. The ln_prev_p pointer of the element's successor in the list points to the
 *      element
 *   4. The number of elements of the containing list, must be at least 1.
 */
void
glist_elem_check_invariants(struct glist_node *elem_p)
{
    FDC_ASSERT_VALID_RAM_POINTER(elem_p, sizeof(void *));

    if (elem_p->ln_anchor_p == NULL) {
        FDC_ASSERT(
            elem_p->ln_next_p == elem_p,
            elem_p->ln_next_p,
            elem_p);

        FDC_ASSERT(
            elem_p->ln_prev_p == elem_p,
            elem_p->ln_prev_p,
            elem_p);
    } else {
        FDC_ASSERT(
            elem_p->ln_next_p != elem_p,
            elem_p->ln_next_p,
            elem_p);

        FDC_ASSERT(
            elem_p->ln_prev_p != elem_p,
            elem_p->ln_prev_p,
            elem_p);

        FDC_ASSERT(
            elem_p->ln_prev_p->ln_next_p == elem_p,
            elem_p->ln_prev_p->ln_next_p,
            elem_p);

        FDC_ASSERT(
            elem_p->ln_next_p->ln_prev_p == elem_p,
            elem_p->ln_next_p->ln_prev_p,
            elem_p);

        FDC_ASSERT(
            elem_p->ln_anchor_p->ln_node_count != 0,
            elem_p->ln_anchor_p,
            elem_p);
    }

} /* glist_elem_check_invariants */

#endif /* _RELIABILITY_CHECKS_ */

