#ifndef LINKED_LIST_HEADER
#define LINKED_LIST_HEADER

template <class T>
class Node;

template <class T>
class ListIterator {
private:
	Node<T>* next_node;
public:
	ListIterator(Node<T>* start_node);
	T next();
	bool hasNext();
};

template <class T>
class LinkedList {
private:
	int count = 0;
	Node<T>* first;
	Node<T>* last;
	Node<T>* getNode(int pos);
	void addNode(Node<T> node);
public:
	unsigned int size();
	void append(T value);
	T &operator[] (int);
	ListIterator<T> getIterator();
	~LinkedList();
};

#endif
