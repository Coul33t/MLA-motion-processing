template<typename Node>
MLATree<Node>::~MLATree() {

}

template<typename Node>
MLATree<Node>::MLATree(const MLATree<Node>& tree) {
	m_graph = tree.m_graph;
}

template<typename Node>
MLATree<Node>& MLATree<Node>::operator=(const MLATree<Node>& tree){
	m_graph = tree.m_graph;
	return *this;
}

template<typename Node>
const bool MLATree<Node>::operator==(const MLATree<Node>& tree) const {
	return m_graph == tree;
}

template<typename Node>
const bool MLATree<Node>::operator!=(const MLATree<Node>& tree) const {
	return !(*this==tree);
}

template<typename Node>
Node* MLATree<Node>::getNode(const unsigned int& idx) const {
	return m_graph.getNodeValue(idx);
}

template<typename Node>
Node* MLATree<Node>::getNode(const std::string& name) const {
	return m_graph.getNodeValue(name);
}

template<typename Node>
void MLATree<Node>::insertNode(Node* node, const unsigned int& parent_idx) {
	m_graph.addChild(parent_idx, node);
}

template<typename Node>
void MLATree<Node>::insertNode(Node* node, const std::string& parent_name) {
	m_graph.addChild(parent_name, node);
}

template<typename Node>
const unsigned int MLATree<Node>::NodeNumber(void) const {
	return m_graph.getSizeGraph();
}