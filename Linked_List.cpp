#include "Linked_List.hpp"
#include <cstddef>

template <class T>
class Node {
private:
	T value;
	Node<T>* next = NULL;
public:
	Node(T value);
	T getValue();
	void setNext(Node<T>* next);
	Node<T>* getNext();
};

template <class T>
Node<T>::Node(T value){
	this->value = value;
}

template <class T>
T Node<T>::getValue(){
	return value;
}

template <class T>
Node<T>*  LinkedList<T>::getNode(int pos){
	if (pos >= count){
		return NULL;
	}

	Node<T>* current = first;
	for (int i = 0; i < pos; i++){
		current = current->getNext();
	}
	return current;
}

template <class T>
T &LinkedList<T>::operator[] (int pos){
	Node<T>* node = getNode(pos);
	return node->getValue();
}

template <class T>
void LinkedList<T>::addNode(Node<T> node){
	if (count == 0){
		first = node;
		last = node;
	} else {
		last->setNext(node);
	}
	count++;
}

template <class T>
void LinkedList<T>::append(T value){
	Node<T> node = new Node<T>(value);
	addNode(node);
}

template <class T>
unsigned int LinkedList<T>::size(){
	return count;
}


template <class T>
ListIterator<T> LinkedList<T>::getIterator(){
	return ListIterator<T>();
}

template <class T>
LinkedList<T>::~LinkedList(){
	Node<T> node = first;
	Node<T> next;
	while(first != NULL){
		next = first.getNext();
		delete first;
		first = next;
	}
}

template <class T>
ListIterator<T>::ListIterator(Node<T>* start_node){
	next_node = start_node;
}

template <class T>
T ListIterator<T>::next(){
	T value = next_node.getValue();
	next_node = next_node.getNext();
	return value;
}

template <class T>
bool ListIterator<T>::hasNext(){
	return next_node != NULL;
}
