"""LAN access to the web (run_web.sh --lan): TCP relays from every address to the local servers, plain or with TLS
(the certificate of the local CA), so the running robot session's container stays as it is (Docker publishes
rosbridge on 127.0.0.1:9090 only) and Flask keeps plain http for the container.
  python3 lan_relay.py --route 9091:9090 --route 5443:5050:tls --route 9443:9090:tls --cert server.crt --key server.key"""
import argparse
import asyncio
import ssl


async def pipe(reader, writer):
    try:
        while data := await reader.read(65536):
            writer.write(data)
            await writer.drain()
    except OSError:
        pass
    finally:
        writer.close()


def relay(target):
    async def connect(reader, writer):
        try:
            upstream = await asyncio.open_connection('127.0.0.1', target)
        except OSError:
            writer.close()
            return
        await asyncio.gather(pipe(reader, upstream[1]), pipe(upstream[0], writer))
    return connect


def route(text):
    """<port>:<target>[:tls]"""
    port, target, *tls = text.split(':')
    return int(port), int(target), tls == ['tls']


async def serve(routes, cert, key):
    context = None
    if any(tls for _, _, tls in routes):
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(cert, key)
    servers = []
    for port, target, tls in routes:
        servers.append(await asyncio.start_server(relay(target), '0.0.0.0', port, ssl=context if tls else None))
        print(f'relay 0.0.0.0:{port} -> 127.0.0.1:{target}{" (TLS)" if tls else ""}', flush=True)
    await asyncio.gather(*(server.serve_forever() for server in servers))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--route', type=route, action='append', required=True)
    parser.add_argument('--cert')
    parser.add_argument('--key')
    options = parser.parse_args()
    asyncio.run(serve(options.route, options.cert, options.key))
