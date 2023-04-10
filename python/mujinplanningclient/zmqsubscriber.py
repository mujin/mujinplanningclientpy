# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 MUJIN Inc

import threading

from . import _
from . import zmq
from . import GetMonotonicTime
from . import TimeoutError, UserInterrupt

import logging
log = logging.getLogger(__name__)


class ZmqSubscriber(object):
    """Subscriber that can handle ongoing subscriptions and automatic socket recreation.
    """

    _ctx = None # zmq context
    _ctxown = None # created zmq context

    _endpoint = None # zmq subscription endpoint
    _endpointFn = None # function that returns the endpoint string to subscribe to, must be thread-safe
    _callbackFn = None # function to call back when new message is received on the subscription socket

    _socket = None # zmq socket
    _lastReceivedTimestamp = 0 # when message was last received on this subscription
    _timeout = 4.0 # beyond this number of seconds, the socket is considered dead and should be recreated
    _checkpreemptfn = None # function for checking for preemptions

    def __init__(self, endpoint, callbackFn=None, timeout=4.0, ctx=None, checkpreemptfn=None):
        """Subscribe to zmq endpoint.

        Args:
            endpoint: the zmq endpoint string to subscribe to, or a thread-safe function that returns such string
            callbackFn: the function to call when subscription receives latest message, it is up to the caller to decode the raw zmq message received, the argument for the callback is the received raw message
            timeout: number of seconds, after this duration, the subscription socket is considered dead and will be recreated automatically to handle network changes (Default: 4.0)
            checkpreemptfn: The function for checking for preemptions
        """
        self._timeout = timeout
        if callable(endpoint):
            self._endpointFn = endpoint
        else:
            self._endpoint = endpoint
        self._callbackFn = callbackFn

        self._ctx = ctx
        if self._ctx is None:
            self._ctxown = zmq.Context()
            self._ctx = self._ctxown

    def __del__(self):
        self.Destroy()

    def __repr__(self):
        return '<%s(endpoint=%r)>' % (self.__class__.__name__, self.endpoint)

    @property
    def endpoint(self):
        """Endpoint string for the subscription
        """
        return self._endpoint

    def Destroy(self):
        self._CloseSocket()
        if self._ctxown is not None:
            try:
                self._ctxown.destroy()
            except Exception as e:
                log.exception('caught exception when destroying zmq context: %s', e)
            self._ctxown = None
        self._ctx = None

    def _HandleReceivedMessage(self, message):
        if self._callbackFn:
            self._callbackFn(message)

    def _HandleTimeout(self, elapsedTime):
        if self._callbackFn:
            self._callbackFn(None)
        raise TimeoutError(_('Timed out waiting to receive message from subscription to "%s" after %0.3f seconds') % (self._endpoint, elapsedTime))

    def _OpenSocket(self):
        # close previous socket just in case
        self._CloseSocket()

        # create new subscription socket with new endpoint
        socket = self._ctx.socket(zmq.SUB)
        socket.setsockopt(zmq.CONFLATE, 1) # store only newest message. have to call this before connect
        socket.setsockopt(zmq.TCP_KEEPALIVE, 1) # turn on tcp keepalive, do these configuration before connect
        socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 2) # the interval between the last data packet sent (simple ACKs are not considered data) and the first keepalive probe; after the connection is marked to need keepalive, this counter is not used any further
        socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 2) # the interval between subsequential keepalive probes, regardless of what the connection has exchanged in the meantime
        socket.setsockopt(zmq.TCP_KEEPALIVE_CNT, 2) # the number of unacknowledged probes to send before considering the connection dead and notifying the application layer
        socket.connect(self._endpoint)
        socket.setsockopt(zmq.SUBSCRIBE, b'') # have to use b'' to make python3 compatible
        self._socket = socket

    def _CloseSocket(self):
        if self._socket:
            try:
                self._socket.close()
            except Exception as e:
                log.exception('failed to close subscription socket for endpoint "%s": %s', self._endpoint, e)
        self._socket = None

    def SpinOnce(self, timeout=None, checkpreemptfn=None):
        """Spin subscription once, ensure that each subscription is received at least once. Block up to supplied timeout duration. If timeout is None, then receive what we can receive without blocking or raising any timeout error.

        Args:
            timeout: If not None, will raise TimeoutError if not all subscriptions can be handled in time. If None, then receive what we can receive without blocking or raising TimeoutError.
            checkpreemptfn: The function for checking for preemptions

        Return:
            Raw message received
        """
        checkpreemptfn = checkpreemptfn or self._checkpreemptfn
        starttime = GetMonotonicTime()

        # check and see if endpoint has changed
        endpoint = self._endpoint
        if self._endpointFn is not None:
            endpoint = self._endpointFn()
        if self._endpoint != endpoint:
            if self._socket is not None:
                log.debug('subscription endpoint changed "%s" -> "%s", so closing previous subscription socket', self._endpoint, endpoint)
            self._CloseSocket()
            self._endpoint = endpoint

        if self._socket is None:
            self._OpenSocket()
            self._lastReceivedTimestamp = starttime

        while True:
            now = GetMonotonicTime()

            # loop to get all received message and only process the last one
            message = None
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
                self._HandleReceivedMessage(message)
                return message

            if now - self._lastReceivedTimestamp > self._timeout:
                self._CloseSocket()
                self._OpenSocket()
                self._lastReceivedTimestamp = now

            if timeout is None:
                return None

            # check for timeout
            if now - starttime > timeout:
                self._HandleTimeout(now - starttime)
                return None

            # check preempt
            if checkpreemptfn:
                checkpreemptfn()

            # poll
            self._socket.poll(20) # poll a little and try again


class ZmqThreadedSubscriber(ZmqSubscriber):
    """A threaded version of ZmqSubscriber.
    """

    _threadName = None # thread name for the background thread
    _thread = None # a background thread for handling subscription
    _stopThread = False # flag to preempt the background thread

    def __init__(self, endpoint, callbackFn=None, timeout=4.0, ctx=None, checkpreemptfn=None, threadName='zmqSubscriber'):
        self._threadName = threadName
        super(ZmqThreadedSubscriber, self).__init__(endpoint, callbackFn=callbackFn, timeout=timeout, ctx=ctx, checkpreemptfn=checkpreemptfn)

        self._StartSubscriberThread()

    def Destroy(self):
        self._StopSubscriberThread()

        super(ZmqThreadedSubscriber, self).Destroy()

    def _StartSubscriberThread(self):
        self._StopSubscriberThread()

        self._stopThread = False
        self._thread = threading.Thread(name=self._threadName, target=self._RunSubscriberThread)
        self._thread.start()

    def _StopSubscriberThread(self):
        self._stopThread = True
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def _CheckPreempt(self):
        if self._stopThread:
            raise UserInterrupt
        if self._checkpreemptfn is not None:
            self._checkpreemptfn()

    def _RunSubscriberThread(self):
        loggedTimeoutError = False # whether time out error has been logged once already
        try:
            while not self._stopThread:
                try:
                    self.SpinOnce(timeout=self._timeout, checkpreemptfn=self._CheckPreempt)
                    loggedTimeoutError = False
                except UserInterrupt:
                    break # preempted
                except TimeoutError as e:
                    if not loggedTimeoutError:
                        log.exception('timed out in subscriber thread: %s', e)
                        loggedTimeoutError = True
                except Exception as e:
                    log.exception('exception caught in subscriber thread: %s', e)
        except Exception as e:
            log.exception('exception caught in subscriber thread: %s', e)
