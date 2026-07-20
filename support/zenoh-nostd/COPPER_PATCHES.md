# Copper patches

This directory is an in-tree fork of `eclipse-zenoh/zenoh-nostd` at
`e88f73a00c9825f5467918c271ee8f1b67f0f771`.

Copper adds a no-alloc, client-only `SerialLinkManager`.  A board constructs an
`EmbeddedIOLink` from its asynchronous UART halves, gives it to the manager,
and connects through a `serial/...` endpoint.  The manager intentionally takes
the UART exactly once and does not implement listener mode.

Upstream issue #11 is not patched by a lifetime-erasing workaround here.  The
Copper integration uses static Embassy-owned session resources until the
upstream resource/transport ownership model is corrected.  Keep any future
upstream fix as a small, separately documented patch.
