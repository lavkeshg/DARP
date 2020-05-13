class Node():
    '''The Node'''

    def __init__(self, key, bus=None, value=None, time=None):
        self.next = None
        self.prev = None
        self.key = key
        self.value = value
        self.bus = bus
        self.time = time
        self.load = None

class LinkedList():
    '''
    Creates a LinkedList
    Formulates the chain of a route and updates the nodes on
    a route with a change in any given node, with the update function
    '''

    def __init__(self):
        self.head = None

    def __len__(self):
        iter = 0
        node = self.head
        nodes = []
        while node is not None:
            nodes.append(str(node.key))
            node = node.next
        # nodes.append('None')
        return len(nodes)

    def __repr__(self):
        node = self.head
        nodes = []
        while node is not None:
            nodes.append(str(node.key))
            node = node.next
        # nodes.append('None')
        return '-->'.join(nodes)

    def __iter__(self):
        node = self.head
        while node is not None:
            yield node.key
            node = node.next

    def __getitem__(self, i):
        node = self.head
        while node is not None:
            if node.key == i:
                return node
            node = node.next
        raise ValueError('Key not found in the list')

    def index(self, i):
        iter = 0
        node = self.head
        while node is not None and node.key != i:
            node = node.next
            iter += 1
        if node is not None:
            return iter
        else:
            raise ValueError('Not in the list')

    def append(self, element: Node):
        if self.head is not None:
            node = self.head
            while node.next is not None:
                node = node.next
            node.next = element
            element.next = None
        else:
            self.head = element

    def copy(self):
        if self.head is not None:
            list = LinkedList()
            list.head = Node(self.head.key, self.head.bus, self.head.value, self.head.time)
            node = self.head
            temp = list.head
            while node is not None:
                if node.next is not None:
                    temp.next = Node(node.next.key, node.next.bus, node.next.value, node.next.time)
                temp = temp.next
                node = node.next
            return list

    def list(self):
        node = self.head
        nodes = []
        while node is not None:
            nodes.append(node.key)
            node = node.next
        return nodes

    def remove(self, remove):
        if self.head is None:
            raise ValueError('List is empty')
        elif remove == self.head.key:
            self.head = self.head.next
        else:
            prev = self.head
            node = prev.next
            while node is not None:
                if node.key == remove:
                    prev.next = node.next
                    node.next = None
                    break
                prev = node
                node = node.next
