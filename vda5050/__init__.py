"""Shared VDA5050 v2.1.0 message contract used by both the master and the client.

Neither side builds topic names, headers or message dicts by hand; they go
through this package so the two programs cannot drift apart.
"""
