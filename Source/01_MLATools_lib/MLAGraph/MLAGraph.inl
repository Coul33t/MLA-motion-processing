


template<typename T>
MLANode<T>::MLANode(const unsigned int& parent, const T value, const std::string& name) :nodeValue(value), parent(parent), name(name){}

template<typename T>
MLANode<T>::MLANode() : nodeValue(T()), parent(0), name(""){}

template<typename T>
MLANode<T>::MLANode(const MLANode<T>&  node) : nodeValue(node.nodeValue), parent(node.parent), name(node.name){}


template<typename T>
MLANode<T>& MLANode<T>::operator = (const MLANode<T>&  node){
	nodeValue = node.nodeValue;
	parent = node.parent;
	name = node.name;
	return *this;
}

template<typename T>
const bool MLANode<T>::operator==(const MLANode<T>& node) const {
	return (parent == node.parent && child == node.child && nodeValue == node.nodeValue && name == node.name);
}

template<typename T>
const bool MLANode<T>::operator!=(const MLANode<T>& node) const {
	return !(*this == node);
}

template<typename T>
MLAGraph<T>::MLAGraph() {};

template<typename T>
MLAGraph<T>::~MLAGraph() {};


//PRB  "struct MLANode<N> MLAGraph<N,E>::GetNode(int idx) {" does not compile with visual
//need a copy constructor and = operator for each template object
//need to return a reference
//Danger: no control on the table size
//index is signed
//template<typename N>
//typename MLANode<N> MLAGraph<N, E>::GetNode(int idx) {
//	return m_nodes[idx];
//};


template<typename T>
MLANode<T>*  MLAGraph<T>::getNode(const unsigned int& idx) {
	if (idx < m_nodes.size())
		return &m_nodes[idx];

	MlaECoutLine("Index out of table");
	return 0;
};

template<typename T>
MLANode<T>*  MLAGraph<T>::getNode(const std::string& name) {
	std::map<std::string, unsigned int idx>::const_iterator iterator = m_names.find(name);

	if (iterator != m_names.end())
		return &m_nodes[iterator->second];

	MlaECoutLine("Index out of table");
	return 0;
}


template<typename T>
T* MLAGraph<T>::getNodeValue(const unsigned int& idx) {
	if (idx < m_nodes.size())
		return &m_nodes[idx].nodeValue;

	MlaECoutLine("Index out of table");
	return 0;
};

template<typename T>
T* MLAGraph<T>::getNodeValue(const std::string& name)  {
	std::map<std::string, unsigned int idx>::const_iterator iterator = m_names.find(name);

	if (iterator != m_names.end())
		return &m_nodes[iterator->second].nodeValue;

	MlaECoutLine("Index not in table");
	return 0;
}

//Danger: no control on the table size
//index is signed
template<typename T>


MLAGraph<T>::MLAGraph(const MLAGraph<T>& graph) {
	m_nodes = graph.m_nodes;
}

template<typename T>
MLAGraph<T>& MLAGraph<T>::operator=(const MLAGraph<T>& graph){
	m_nodes = graph.m_nodes;
	return *this;
}

template<typename T>
const bool MLAGraph<T>::operator==(const MLAGraph<T>& graph) const {
	return(m_nodes == graph.m_nodes && m_names == graph.m_names);
}

template<typename T>
const bool MLAGraph<T>::operator!=(const MLAGraph<T>& graph) const {
	return !(*this == graph);
}


//template<typename T>
//T& MLAGraph<T>::getNodeValues(const unsigned int& idx) {
//	static T dummyNodeValue;
//	if (idx < m_nodes.size()) { return m_nodes[idx].nodeValue; }
//	MlaECoutLine("Index out of table");
//	return dummyNodeValue;
//};


template<typename T>
const unsigned int MLAGraph<T>::getSizeGraph() const{
	return m_nodes.size();
}

template<typename T>
void MLAGraph<T>::setChild(const int& parent_idx, const unsigned int& child_idx) {
	if (parent_idx< (int)m_nodes.size() && child_idx < m_nodes.size()) { m_nodes[parent_idx].child.push_back(child_idx); return; }
	MlaECoutLine("Index out of table");
}

//need a copy constructor and = operator for each template object
//addRoot and addChild can be simplify with a unique procedure addNode
template<typename T>
void MLAGraph<T>::addRoot(const T& node) {
	MLANode<T> new_node;
	new_node.nodeValue = node;
	new_node.parent = -1;
	m_nodes.push_back(new_node);
};

//need a copy constructor and = operator for each template object
//addRoot and addChild can be simplify with a unique procedure addNode
template<typename T>
void MLAGraph<T>::addChild(const int& parent, const T& node) {
	MLANode<T> new_node;
	new_node.nodeValue = node;
	new_node.parent = parent;
	m_names[node.parent] = m_nodes.size();
	m_nodes.push_back(new_node);
};

template<typename T>
void addChild(const std::string& parent, const T& node) {
	MLANode<T> new_node;
	new_node.nodeValue = node;
	if (getNode(parent)){
		new_node.parent=m_names[parent];
	}
	m_names[node.parent] = m_nodes.size();
	m_nodes.push_back(new_node);
};


