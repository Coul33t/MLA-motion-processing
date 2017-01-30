template<typename T>
MlaNode<T>::MlaNode(const unsigned int& parent, const T value) :nodeValue(value), parent(parent){}

template<typename T>
MlaNode<T>::MlaNode() : nodeValue(T()), parent(0){}

template<typename T>
MlaNode<T>::MlaNode(const MlaNode<T>&  node) : nodeValue(node.nodeValue), parent(node.parent){}


template<typename T>
MlaNode<T>& MlaNode<T>::operator = (const MlaNode<T>&  node){
	nodeValue = node.value;
	parent = node.parent;
}

template<typename T>
MlaGraph<T>::MlaGraph() {};

template<typename T>
MlaGraph<T>::~MlaGraph() {};


//PRB  "struct MlaNode<N> MlaGraph<N,E>::GetNode(int idx) {" does not compile with visual
//need a copy constructor and = operator for each template object
//need to return a reference
//Danger: no control on the table size
//index is signed
//template<typename N>
//typename MlaNode<N> MlaGraph<N, E>::GetNode(int idx) {
//	return m_nodes[idx];
//};


template<typename T>
const MlaNode<T>& MlaGraph<T>::getNode(const unsigned int& idx) const{
	const static MlaNode<T> dummyNode;
	if (idx < m_nodes.size()) { return m_nodes[idx]; }
	MlaECoutLine("Index out of table");
	return dummyNode;
};

//need a copy constructor and = operator for each template object
//Danger: no control on the table size
//index is signed
template<typename T>
T& MlaGraph<T>::getNodeValues(const unsigned int& idx) {
	static T dummyNodeValue;
	if (idx < m_nodes.size()) { return m_nodes[idx].nodeValue; }
	MlaECoutLine("Index out of table");
	return dummyNodeValue;
};


template<typename T>
const unsigned int MlaGraph<T>::getSizeNode() const{
	return m_nodes.size();
}

template<typename T>
void MlaGraph<T>::setChild(const int& parent_idx, const unsigned int& child_idx) {
	if (parent_idx< (int)m_nodes.size() && child_idx < m_nodes.size()) { m_nodes[parent_idx].child.push_back(child_idx); return; }
	MlaECoutLine("Index out of table");
}

//need a copy constructor and = operator for each template object
//addRoot and addChild can be simplify with a unique procedure addNode
template<typename T>
void MlaGraph<T>::addRoot(const T& node) {
	MlaNode<T> new_node;
	new_node.nodeValue = node;
	new_node.parent = -1;
	m_nodes.push_back(new_node);
};

//need a copy constructor and = operator for each template object
//addRoot and addChild can be simplify with a unique procedure addNode
template<typename T>
void MlaGraph<T>::addChild(const int& parent, const T& node) {
	MlaNode<T> new_node;
	new_node.nodeValue = node;
	new_node.parent = parent;
	m_nodes.push_back(new_node);
};



