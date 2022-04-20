


#include <csp/include/csp/csp.h>
#include <csp/include/csp/csp_debug.h>
#include <string.h>


int csp_sys_tasklist(char * out) {

	strcpy(out, "Tasklist not available on OSX");
	return CSP_ERR_NONE;
}

int csp_sys_tasklist_size(void) {

	return 100;
}

uint32_t csp_sys_memfree(void) {

	return 0;  // not implemented
}