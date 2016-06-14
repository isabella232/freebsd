/*-
 * Copyright (c) 2016 Jakub Klama <jceel@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY NETAPP, INC ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL NETAPP, INC OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/types.h>
#include <sys/queue.h>
#include <sys/event.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>

#include <sys/nv.h>
#include "ipc.h"

#define	WPRINTF(params)	printf params

struct ipc_service;
struct ipc_method;
struct ipc_call;

struct ipc_service
{
	const char *		is_name;
	void *			is_softc;
	LIST_HEAD(, ipc_method)	is_methods;
	LIST_ENTRY(ipc_service)	is_link;
};

struct ipc_method
{
	struct ipc_service *	im_service;
	const char *		im_name;
	ipc_func_t		im_func;
	LIST_ENTRY(ipc_method)	im_link;
};

struct ipc_call
{
	int			ic_id;
	const char *		ic_service;
	const char *		ic_method;
	nvlist_t *		ic_nvl;
	LIST_ENTRY(ipc_call)	ic_link;
};

struct ipc_softc {
	int			ipc_fd;
	int			ipc_cfd;
	bool			ipc_events;
	pthread_t		ipc_thread;
	pthread_mutex_t		ipc_wlock;
	pthread_mutex_t		ipc_waitmtx;
	pthread_cond_t		ipc_waitcond;
	LIST_HEAD(, ipc_service) ipc_services;
	LIST_HEAD(, ipc_call)	ipc_calls;
};

static struct ipc_softc ipc;

static void ipc_handle_call(int);
static int ipc_handle_client(int);
static void *ipc_handle(void *);
static void ipc_send(nvlist_t *);

struct ipc_service *
ipc_add_service(const char *name, void *softc)
{
	struct ipc_service *svc;

	svc = calloc(1, sizeof(struct ipc_service));
	if (svc == NULL)
		return (NULL);

	svc->is_name = name;
	svc->is_softc = softc;
	LIST_INIT(&svc->is_methods);
	LIST_INSERT_HEAD(&ipc.ipc_services, svc, is_link);

	return (svc);
}

struct ipc_method *
ipc_add_method(struct ipc_service *svc,
    const char *name, ipc_func_t handler)
{
	struct ipc_method *method;

	method = calloc(1, sizeof(struct ipc_method));
	if (method == NULL)
		return (NULL);

	method->im_service = svc;
	method->im_name = name;
	method->im_func = handler;
	LIST_INSERT_HEAD(&svc->is_methods, method, im_link);

	return (method);
}

int
ipc_delete_service(struct ipc_service *service)
{

	return (-1);
}

void
ipc_send_event(struct ipc_service *service, const char *name,
    nvlist_t *payload)
{
	nvlist_t *nvl;

	nvl = nvlist_create(NV_FLAG_IGNORE_CASE);
	nvlist_add_string(nvl, "service", service->is_name);
	nvlist_add_string(nvl, "event", name);

	if (payload != NULL)
		nvlist_add_nvlist(nvl, "payload", payload);

	ipc_send(nvl);	
}

void
ipc_respond_ok(void *id, nvlist_t *response)
{
	struct ipc_call *call;
	nvlist_t *nvl;

	assert(id != NULL);

	call = (struct ipc_call *)id;
	nvl = nvlist_create(NV_FLAG_IGNORE_CASE);
	nvlist_add_number(nvl, "id", call->ic_id);
	nvlist_add_number(nvl, "error", 0);

	if (response != NULL)
		nvlist_move_nvlist(nvl, "response", response);

	ipc_send(nvl);	
}

void
ipc_respond_err(void *id, int errnum, const char *errstr)
{
	struct ipc_call *call;
	nvlist_t *nvl;

	call = (struct ipc_call *)id;
	nvl = nvlist_create(NV_FLAG_IGNORE_CASE);
	nvlist_add_number(nvl, "id", call != NULL ? call->ic_id : 0);
	nvlist_add_number(nvl, "error", errnum);
	nvlist_add_string(nvl, "errstr", errstr);

	ipc_send(nvl);
}

static void
ipc_handle_call(int cfd)
{
	struct ipc_service *svc;
	struct ipc_method *method;
	struct ipc_call *call;
	nvlist_t *nvl;

	method = NULL;
	nvl = nvlist_recv(cfd, 0);
	if (nvl == NULL) {
		WPRINTF(("failed to receive nvlist from IPC socket\n"));
		return;
	}

	if (!nvlist_exists_number(nvl, "id")) {
		ipc_respond_err(NULL, EINVAL, "'id' missing");
		return;
	}

	if (!nvlist_exists_string(nvl, "service")) {
		ipc_respond_err(NULL, EINVAL, "'service' missing");
		return;
	}

	if (!nvlist_exists_string(nvl, "method")) {
		ipc_respond_err(NULL, EINVAL, "'method' missing");
		return;
	}

	if (!nvlist_exists_nvlist(nvl, "args")) {
		ipc_respond_err(NULL, EINVAL, "'args' missing");
		return;
	}

	call = calloc(1, sizeof(struct ipc_call));
	call->ic_id = nvlist_get_number(nvl, "id");
	call->ic_service = nvlist_get_string(nvl, "service");
	call->ic_method = nvlist_get_string(nvl, "method");
	call->ic_nvl = nvl;

	LIST_FOREACH(svc, &ipc.ipc_services, is_link) {
		if (strcmp(svc->is_name, call->ic_service) != 0)
			continue;

		LIST_FOREACH(method, &svc->is_methods, im_link) {
			if (strcmp(method->im_name, call->ic_method) == 0)
				break;
		}
	}

	if (method == NULL) {
		ipc_respond_err(call, ENOENT, "Service or method not found");
		return;
	}

	method->im_func(svc, (void *)call, nvlist_get_nvlist(nvl, "args"));
}

static void *
ipc_handle(void *arg)
{

	for (;;) {
		ipc.ipc_cfd = accept(ipc.ipc_fd, NULL, NULL);
		if (ipc.ipc_cfd < 0)
			return (NULL);

		ipc_handle_client(ipc.ipc_cfd);
	}
}

static int
ipc_handle_client(int cfd)
{
	struct kevent kev, ret;
	int kq;
	int err;

	kq = kqueue();

	EV_SET(&kev, cfd, EVFILT_READ, EV_ADD | EV_ENABLE, 0, 0, NULL);

	err = kevent(kq, &kev, 1, NULL, 0, NULL);
	if (err < 0)
		goto out;

	for (;;) {
		err = kevent(kq, NULL, 0, &ret, 1, NULL);
		if (err < 0)
			goto out;

		if (ret.flags & EV_EOF)
			goto out;

		if (ret.flags & EV_ERROR)
			goto out;

		ipc_handle_call(cfd);
	}

out:
	close(kq);
	return (-1);
}

static void
ipc_send(nvlist_t *nvl)
{

	pthread_mutex_lock(&ipc.ipc_wlock);

	if (ipc.ipc_cfd >= 0)
		nvlist_send(ipc.ipc_cfd, nvl);

	pthread_mutex_unlock(&ipc.ipc_wlock);
}

static int
ipc_handle_continue(struct ipc_service *svc, void *id, const nvlist_t *args)
{

	pthread_cond_signal(&ipc.ipc_waitcond);
	ipc_respond_ok(id, NULL);
	return (0);
}

static void
ipc_register_system()
{
	struct ipc_service *svc;

	svc = ipc_add_service("system", NULL);
	ipc_add_method(svc, "continue", ipc_handle_continue);
}

int
ipc_init(char *opts)
{
	char *path;
	struct sockaddr_un sun;
	bool wait;
	int s;

	path = strsep(&opts, ",");
	
	if (opts != NULL && strcmp(opts, "wait") == 0)
		wait = true;

	s = socket(AF_UNIX, SOCK_STREAM, 0);
	if (s < 0)
		return (-1);

	sun.sun_family = AF_UNIX;
	sun.sun_len = sizeof(struct sockaddr_un);
	strncpy(sun.sun_path, path, sizeof(sun.sun_path));

	if (bind(s, (struct sockaddr *)&sun, sun.sun_len) < 0)
		goto out;

	if (listen(s, 5) < 0)
		goto out;

	ipc.ipc_fd = s;
	ipc.ipc_cfd = -1;
	pthread_mutex_init(&ipc.ipc_wlock, NULL);
	pthread_mutex_init(&ipc.ipc_waitmtx, NULL);
	pthread_cond_init(&ipc.ipc_waitcond, NULL);
	LIST_INIT(&ipc.ipc_services);
	LIST_INIT(&ipc.ipc_calls);

	if (pthread_create(&ipc.ipc_thread, NULL, ipc_handle, NULL) < 0)
		goto out;

	ipc_register_system();

	if (wait) {
		pthread_mutex_lock(&ipc.ipc_waitmtx);
		pthread_cond_wait(&ipc.ipc_waitcond, &ipc.ipc_waitmtx);
		pthread_mutex_unlock(&ipc.ipc_waitmtx);
	}

	return (0);

out:
	close(s);
	return (-1);
}

