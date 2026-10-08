"""Complete latest-sample consent snapshots: both ports in one message."""
from .shapes import edges


def connections_for(role, owners, closed, peers, bonds):
    connected = {(bond['a'], bond['b']) for bond in bonds}
    consents = []
    for a, b in edges(len(owners), closed):
        if role not in (a, b) or (owners[a], owners[b]) in connected:
            continue
        neighbor = owners[b if role == a else a]
        if peers.get(neighbor, {}).get('arrived'):
            consents.append(dict(a=owners[a], b=owners[b]))
    return consents
