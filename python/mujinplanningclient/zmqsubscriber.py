# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 MUJIN Inc

import weakref

from . import _
from . import zmq
from . import GetMonotonicTime
from . import TimeoutError

import logging
log = logging.getLogger(__name__)

class ZmqSubscription(object):

    _subscriber = None # weakref to ZmqSubscriber
    _socket = None # zmq socket
    _lastReceivedTimestamp = 0 # when message was last received on this subscription

    _endpoint = None # zmq subscription endpoint
    _callbackFn = None # function to call back when new message is received on the subscription socket
    _timeout = 4.0 # beyond this number of seconds, the socket is considered dead and should be recreated

    def __del__(self):
        self.Unsubscribe()

    def Unsubscribe(self):
        """Stop the subscription.
        """
        if self._subscriber:
            self._subscriber.Unsubscribe(self)
            self._subscriber = None
        self._CloseSocket()

    @property
    def endpoint(self):
        return self._endpoint

    def _EnsureSocket(self, ctx, now):
        if self._socket:
            return
        socket = ctx.socket(zmq.SUB)
        socket.setsockopt(zmq.CONFLATE, 1) # store only newest message. have to call this before connect
        socket.setsockopt(zmq.TCP_KEEPALIVE, 1) # turn on tcp keepalive, do these configuration before connect
        socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 2) # the interval between the last data packet sent (simple ACKs are not considered data) and the first keepalive probe; after the connection is marked to need keepalive, this counter is not used any further
        socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 2) # the interval between subsequential keepalive probes, regardless of what the connection has exchanged in the meantime
        socket.setsockopt(zmq.TCP_KEEPALIVE_CNT, 2) # the number of unacknowledged probes to send before considering the connection dead and notifying the application layer
        socket.connect(self._endpoint)
        socket.setsockopt(zmq.SUBSCRIBE, b'') # have to use b'' to make python3 compatible
        self._socket = socket
        self._lastReceivedTimestamp = now

    def _CloseSocket(self):
        if self._socket:
            try:
                self._socket.close()
            except Exception as e:
                log.exception('failed to close subscription socket for endpoint "%s": %s', self._endpoint, e)
        self._socket = None

    def _TryReceiveOnce(self, now):
        message = None
        # loop to get all received message and only process the last one
        while True:
            try:
                message = self._socket.recv(zmq.NOBLOCK)
            except zmq.ZMQError as e:
                if e.errno != zmq.EAGAIN:
                    log.exception('caught exception while trying to receive from subscription socket for endpoint "%s": %s', self._endpoint, e)
                    self._CloseSocket()
                    raise
                break # got EAGAIN, so break
        if message is not None:
            self._lastReceivedTimestamp = now
            self._callbackFn(self, message)
            return True
        return False

class ZmqSubscriber(object):

    _ctx = None # zmq context
    _ctxown = None # created zmq context
    _poller = None # zmq poller

    _subscriptions = None # a list of ZmqSubscription

    def __init__(self, ctx=None):
        self._subscriptions = []

        self._ctx = ctx
        if self._ctx is None:
            self._ctxown = zmq.Context()
            self._ctx = self._ctxown

        self._poller = zmq.Poller()

    def __del__(self):
        self.Destroy()

    def Destroy(self):
        for subscription in self._subscriptions:
            subscription._subscriber = None
            subscription._CloseSocket()
        self._subscriptions = []
        if self._ctxown is not None:
            try:
                self._ctxown.destroy()
            except Exception as e:
                log.exception('caught exception when destroying zmq context: %s', e)
            self._ctxown = None
        self._ctx = None

    def Subscribe(self, endpoint, callbackFn, timeout=4.0):
        """Subscribe to zmq endpoint
        """
        subscription = ZmqSubscription()
        subscription._timeout = timeout
        subscription._endpoint = endpoint
        subscription._callbackFn = callbackFn
        subscription._subscriber = weakref.proxy(self)
        self._subscriptions.append(subscription)
        return subscription

    def Unsubscribe(self, subscription):
        """Stop a subscription.
        """
        if subscription in self._subscriptions:
            self._subscriptions.remove(subscription)
        subscription._subscriber = None
        subscription._CloseSocket()

    def SpinOnce(self, timeout=None, checkpreemptfn=None):
        """Spin all subscription once, ensure that each subscription is received at least once. Block up to supplied timeout duration. If timeout is None, then receive what we can receive without blocking or raising any timeout error.
        """
        starttime = GetMonotonicTime()

        # make a list of the subscriptions, we will mark them done as we receive messages on them
        subscriptions = list(self._subscriptions)
        for subscription in subscriptions:
            subscription._EnsureSocket(self._ctx, starttime)

        while True:
            now = GetMonotonicTime()

            # try receive for each subscription
            for index, subscription in enumerate(subscriptions):
                if subscription is not None:
                    if subscription._TryReceiveOnce(now):
                        subscriptions[index] = None # already done

            # re-create timed out sockets
            for subscription in subscriptions:
                if subscription:
                    if now - subscription._lastReceivedTimestamp > subscription._timeout:
                        subscription._CloseSocket()
            for subscription in subscriptions:
                if subscription:
                    subscription._EnsureSocket(self._ctx, now)

            if timeout is None:
                return

            # check if all subscriptions have received at least once
            hasPendingSubscription = False
            for subscription in subscriptions:
                if subscription is not None:
                    hasPendingSubscription = True
                    break
            if not hasPendingSubscription:
                return

            # check for timeout
            if now - starttime > timeout:
                raise TimeoutError(_('Timed out waiting for subscriptions'))
            if checkpreemptfn:
                checkpreemptfn()

            # poll what is left
            try:
                for subscription in subscriptions:
                    if subscription is not None and subscription._socket is not None:
                        self._poller.register(subscription._socket, zmq.POLLIN)
                self._poller.poll(20) # poll a little and try again
            finally:
                for subscription in subscriptions:
                    if subscription is not None and subscription._socket is not None:
                        self._poller.unregister(subscription._socket)
