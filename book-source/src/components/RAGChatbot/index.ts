/**
 * RAG Chatbot Component
 * Export for easy importing
 * 
 * Default: RAGChatbot (custom implementation - works WITHOUT workflow)
 * Optional: ChatKitRAG (requires OpenAI ChatKit workflow)
 */

// Export custom RAGChatbot as default (works without workflow)
export { default } from './RAGChatbot';

// Export ChatKit version (requires OpenAI workflow - see CHATKIT_WORKFLOW_GUIDE.md)
export { default as ChatKitRAG } from './ChatKitRAG';
