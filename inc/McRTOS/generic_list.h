/**
 * @file generic_list.h
 *
 * Generic linked lists.
 *
 * @author: German Rivera
 */

#ifndef _GENERIC_LIST_H
#define _GENERIC_LIST_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>		/* offsetof() */

/**
 * A node in a circular doubly-linked list
 */
struct glist_node {
	/**
	 * Forward link
	 */
	struct glist_node *ln_next_p;

	/**
	 * Backward link
	 */
	struct glist_node *ln_prev_p;

#ifdef _RELIABILITY_CHECKS_
	/**
	 * A glist_node structure can be either the anchor node of a list
         * or a child node in a list, but not both.
	 */
	union {
		/**
		 * Pointer to the anchor node (for child nodes)
		 * the element does not belong to a list.
		 */
		struct glist_node *ln_anchor_p;

		/**
		 * Number of nodes in the list (for anchor nodes)
     		 */
		uintptr_t ln_node_count;
	};
#endif /* _RELIABILITY_CHECKS_ */
};


#ifdef _RELIABILITY_CHECKS_

/**
 * Compile time initializer of a list node
 */
#define GLIST_NODE_INITIALIZER(_list_node)				\
 	{ 								\
		.ln_next_p = &(_list_node),				\
		.ln_prev_p = &(_list_node),				\
		.ln_node_count = 0					\
	}

/**
 * Macro to initialize a glist_node structure at run time 
 */
#define GLIST_NODE_INIT(_list_p) 					\
 	do { 								\
		(_list_p)->ln_next_p = (_list_p);			\
		(_list_p)->ln_prev_p = (_list_p);			\
		(_list_p)->ln_node_count = 0;			        \
	} while(0)

/**
 * Return the pointer to the list of which a given list node is an element of or NULL if
 * the list node does not belong to a list.
 *
 * @param _elem_p	variable of type 'struct glist_node *' that points to
 * 			the element.
 */
#define GLIST_NODE_GET_LIST(_elem_p) \
	((_elem_p)->ln_anchor_p) 

/**
 * Return the number of elements of a given list
 *
 * @param _list_p	variable of type 'struct glist_node *' that points to
 * 			the list.
 */
#define GLIST_GET_NODE_COUNT(_list_p) \
	((_list_p)->ln_node_count) 

#else

#define GLIST_NODE_INITIALIZER(_list_node)				\
 	{ 								\
		.ln_next_p = &(_list_node),				\
		.ln_prev_p = &(_list_node),				\
	}

/**
 * Macro to initialize a glist_node structure at run time 
 */
#define GLIST_NODE_INIT(_list_p) 					\
 	do { 								\
		(_list_p)->ln_next_p = (_list_p);			\
		(_list_p)->ln_prev_p = (_list_p);			\
	} while(0)

#endif /* _RELIABILITY_CHECKS_ */

/**
 * Iterator to traverse a list
 *
 * @param _elem_p	variable of type 'struct glist_node *' used as loop index.
 * @param _list_p	variable of type 'struct glist_node *' that points to the
 *			linked list object being traversed.
 */
#define GLIST_FOR_EACH_NODE(_elem_p, _list_p)	\
	for ((_elem_p) = (_list_p)->ln_next_p,	  	\
	     __builtin_prefetch((_elem_p)->ln_next_p);	\
	     (_elem_p) != (_list_p);			\
	     (_elem_p) = (_elem_p)->ln_next_p,		\
	     __builtin_prefetch((_elem_p)->ln_next_p))

/**
 * Iterator to traverse a list when elements are being removed
 * from the list as the list is being traversed.
 *
 * @param _elem_p	variable of type 'struct glist_node *' used as loop index.
 * @param _next_p	variable of type 'struct glist_node *' used as a temp
 *			location to save the ln_next_p forward link between 
 *			iterations.
 * @param _list_p	variable of type 'struct glist_node *' that points to the
 *			linked list object being traversed.
 */
#define GLIST_FOR_EACH_NODE_REMOVING(_elem_p, _next_p, _list_p)		\
	for ((_elem_p) = (_list_p)->ln_next_p,				\
	     (_next_p) = (_elem_p)->ln_next_p;			        \
	     (_elem_p) != (_list_p);					\
	     (_elem_p) = (_next_p),				        \
	     (_next_p) = (_elem_p)->ln_next_p)

#define GLIST_NODE_ENTRY(_elem_p, _enclosing_struct_type, _elem_field) 	\
	ENCLOSING_STRUCT(_elem_p, _enclosing_struct_type, _elem_field)

/**
 * Iterator to traverse a list using a pointer to the element's enclosing
 * structure
 *
 * @param _enclosing_struct_p	pointer to a structure that contains a
 *				field of type 'struct glist_node'.
 * @param _list_p		variable of type 'struct glist_node *' that
 *				points to the linked list object being
 *				traversed.
 * @param _elem_field		name of field of type 'struct glist_node' in
 *				the structure referenced by first arg.
 *
 */
#define GLIST_FOR_EACH_ENTRY(_enclosing_struct_p, _list_p, _elem_field) \
	for ((_enclosing_struct_p) = 					\
			GLIST_NODE_ENTRY((_list_p)->ln_next_p,		\
				   __typeof__(*(_enclosing_struct_p)),	\
				   _elem_field),			\
	     __builtin_prefetch(					\
	     		(_enclosing_struct_p)->_elem_field.ln_next_p);	\
	     &(_enclosing_struct_p)->_elem_field != (_list_p);		\
	     (_enclosing_struct_p) =					\
		GLIST_NODE_ENTRY(					\
			(_enclosing_struct_p)->_elem_field.ln_next_p,	\
			__typeof__(*(_enclosing_struct_p)),		\
			_elem_field),					\
	     __builtin_prefetch(					\
	     		(_enclosing_struct_p)->_elem_field.ln_next_p))

/**
 * Iterator to traverse a list using a pointer to the element's enclosing
 * structure, when elements are being removed from the list as the
 * list is being traversed
 *
 * @param _enclosing_struct_p	pointer to a structure that contains a
 *				field of type 'struct glist_node'.
 * @param _next_entry_p	        pointer of the same type as first arg, used as
 *				a temp location to save the ln_next_p forward
 *				link between iterations.
 * @param _list_p		variable of type 'struct glist_node *' that
 *				points to the linked list object being
 *				traversed.
 * @param _elem_field		name of field of type 'struct glist_node' in
 *				the structure referenced by first arg.

 */
#define GLIST_FOR_EACH_ENTRY_REMOVING(_enclosing_struct_p, _next_entry_p,\
 				 _list_p, _elem_field) 			\
	for ((_enclosing_struct_p) = 					\
			GLIST_NODE_ENTRY((_list_p)->ln_next_p,		\
				   __typeof__(*(_enclosing_struct_p)),	\
				   _elem_field),			\
	     (_next_entry_p) =					        \
		GLIST_NODE_ENTRY(					\
			(_enclosing_struct_p)->_elem_field.ln_next_p,	\
			__typeof__(*(_enclosing_struct_p)),		\
			_elem_field);					\
	     &(_enclosing_struct_p)->_elem_field != (_list_p);		\
	     (_enclosing_struct_p) = (_next_entry_p),		        \
	     (_next_entry_p) =					        \
		GLIST_NODE_ENTRY(					\
			(_enclosing_struct_p)->_elem_field.ln_next_p,	\
			__typeof__(*(_enclosing_struct_p)),		\
			_elem_field))

/**
 * Return the pointer to the first element of a list or NULL if the list
 * is empty.
 *
 * @param _list_p	variable of type 'struct glist_node *' that points to
 * 			the list.
 */
#define GLIST_GET_FIRST(_list_p) \
	(GLIST_IS_EMPTY(_list_p) ? NULL : (_list_p)->ln_next_p) 

/**
 * Return the pointer to the last element of a list or NULL if the list
 * is empty.
 *
 * @param _list_p	variable of type 'struct glist_node *' that points to
 * 			the list.
 */
#define GLIST_GET_LAST(_list_p) \
	(GLIST_IS_EMPTY(_list_p) ? NULL : (_list_p)->ln_prev_p) 

/**
 * Tell if a list node is not in any list
 */
#ifdef DEBUG
#   define GLIST_NODE_IS_UNLINKED(_list_node_p) \
           glist_node_is_unlinked(_list_node_p)
#else
#   define GLIST_NODE_IS_UNLINKED(_list_node_p) \
           ((_list_node_p)->ln_next_p == (_list_node_p))
#endif

/**
 * Tell if a list is empty 
 */
#define GLIST_IS_EMPTY(_list_p)     GLIST_NODE_IS_UNLINKED(_list_p)

/**
 * Tell if a list is not empty 
 */
#define GLIST_IS_NOT_EMPTY(_list_p) (!GLIST_IS_EMPTY(_list_p))     

/**
 * Tell if a list node belongs to a list (any list)
 */
#define GLIST_NODE_IS_LINKED(_list_node_p) \
        (! GLIST_NODE_IS_UNLINKED(_list_node_p))

    
typedef void glist_add_elem_function_t(
                struct glist_node *list_p,
                struct glist_node *elem_p);


bool glist_node_is_unlinked(const struct glist_node *list_node_p);

void glist_add_head_elem(struct glist_node *list_p, struct glist_node *elem_p);

void glist_add_tail_elem(struct glist_node *list_p, struct glist_node *elem_p);

void glist_remove_elem(struct glist_node *elem_p);

void glist_migrate_nodes(
        struct glist_node *src_list_p, struct glist_node *dest_list_p);

#ifdef _RELIABILITY_CHECKS_

void glist_anchor_check_invariants(struct glist_node *list_p);

void glist_elem_check_invariants(struct glist_node *elem_p);

#endif /* _RELIABILITY_CHECKS_ */

#endif /* _GENERIC_LIST_H */
