/**
 * RAG Chatbot Component
 * A floating chatbot interface for RAG-enhanced Q&A about the textbook
 * Features: session persistence, source citations, markdown rendering
 */

import React, { useState, useEffect, useRef, useCallback } from 'react';
import ReactMarkdown from 'react-markdown';
import { MessageCircle, X, Send, Trash2, Loader2, BookOpen, Sparkles } from 'lucide-react';
import type { ChatMessage, SourceCitation } from '../../types/chat';
import { sendChatMessage, sendSelectedTextQuery, getSessionId, clearSessionId, clearChatHistory } from '../../services/chatApi';
import { useTextSelection } from '../../hooks/useTextSelection';
import styles from './styles.module.css';

export default function RAGChatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string>('');
  const [error, setError] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Handle text selection
  const handleTextSelection = (text: string) => {
    setSelectedText(text);
    // Auto-open chatbot when text is selected
    if (!isOpen) {
      setIsOpen(true);
    }
    // Pre-fill input with a helpful prompt
    setInputValue(`Guide me about this content`);
  };

  const { clearSelection } = useTextSelection(handleTextSelection);

  // Initialize session ID on mount
  useEffect(() => {
    if (typeof window !== 'undefined') {
      const id = getSessionId();
      setSessionId(id);
    }
  }, []);

  // Ensure button is always clickable in production - re-attach event listeners if needed
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const button = document.querySelector('[data-chat-button]') as HTMLButtonElement;
    if (button) {
      // Force re-attach click handler as fallback
      const handleClick = (e: Event) => {
        e.preventDefault();
        e.stopPropagation();
        setIsOpen((prev) => !prev);
      };

      button.addEventListener('click', handleClick, { capture: true });
      
      return () => {
        button.removeEventListener('click', handleClick, { capture: true });
      };
    }
  }, []);

  // Auto-scroll to latest message
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const messageText = inputValue.trim();
    const userMessage: ChatMessage = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: messageText,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);
    clearSelection(); // Clear text selection after sending

    try {
      let response;
      
      // If there's selected text, use selected text query endpoint
      if (selectedText && selectedText.trim().length > 0) {
        response = await sendSelectedTextQuery(sessionId, messageText, selectedText);
        setSelectedText(null); // Clear after use
      } else {
        // Regular chat query
        response = await sendChatMessage(sessionId, messageText);
      }

      const assistantMessage: ChatMessage = {
        id: `assistant-${Date.now()}`,
        role: 'assistant',
        content: response.message,
        timestamp: new Date(response.timestamp),
        sources: response.sources,
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Chat error:', err);
      setError('Failed to get response. Please check if the backend is running.');

      // Add error message to chat
      const errorMessage: ChatMessage = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please check if the backend server is accessible.',
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleClearChat = async () => {
    if (!confirm('Clear all chat history?')) return;

    try {
      await clearChatHistory(sessionId);
      setMessages([]);
      clearSessionId();
      const newId = getSessionId();
      setSessionId(newId);
      setError(null);
    } catch (err) {
      console.error('Clear chat error:', err);
      setError('Failed to clear chat history');
    }
  };

  const toggleChat = useCallback(() => {
    setIsOpen((prev) => !prev);
  }, []);

  // Prevent event propagation issues - more robust for production
  const handleButtonClick = (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    e.nativeEvent.stopImmediatePropagation();
    toggleChat();
  };

  const handleMouseDown = (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    e.nativeEvent.stopImmediatePropagation();
  };

  const handleTouchStart = (e: React.TouchEvent) => {
    // Use non-passive listener to allow preventDefault
    e.preventDefault();
    e.stopPropagation();
    toggleChat();
  };

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={styles.floatingButton}
        onClick={handleButtonClick}
        onMouseDown={handleMouseDown}
        aria-label="Toggle chatbot"
        title="Ask questions about the textbook"
        type="button"
        data-chat-button="true"
        style={{
          zIndex: 9999,
          pointerEvents: 'auto',
          position: 'fixed',
          bottom: '2rem',
          right: '2rem',
          touchAction: 'manipulation', // Prevents passive listener warning
        }}
        ref={(button) => {
          // Add non-passive touch event listener directly to DOM
          if (button && typeof window !== 'undefined') {
            button.addEventListener('touchstart', (e) => {
              e.preventDefault();
              e.stopPropagation();
              toggleChat();
            }, { passive: false });
          }
        }}
      >
        {isOpen ? <X size={24} /> : <MessageCircle size={24} />}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <BookOpen size={20} />
              <div>
                <h3 className={styles.headerTitle}>Textbook Assistant</h3>
                <p className={styles.headerSubtitle}>Ask me anything about Physical AI</p>
              </div>
            </div>
            <button
              className={styles.clearButton}
              onClick={handleClearChat}
              title="Clear chat history"
              aria-label="Clear chat"
            >
              <Trash2 size={18} />
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.emptyState}>
                <BookOpen size={48} className={styles.emptyIcon} />
                <h4>Ask me anything!</h4>
                <p>I can help you understand concepts from the Physical AI textbook.</p>
                <div className={styles.exampleQuestions}>
                  <p className={styles.exampleTitle}>Try asking:</p>
                  <button
                    className={styles.exampleButton}
                    onClick={() => setInputValue('What is physical AI?')}
                  >
                    What is physical AI?
                  </button>
                  <button
                    className={styles.exampleButton}
                    onClick={() => setInputValue('How do humanoid robots work?')}
                  >
                    How do humanoid robots work?
                  </button>
                  <button
                    className={styles.exampleButton}
                    onClick={() => setInputValue('Explain reinforcement learning')}
                  >
                    Explain reinforcement learning
                  </button>
                </div>
              </div>
            )}

            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${styles[message.role]}`}
              >
                <div className={styles.messageContent}>
                  <ReactMarkdown>{message.content}</ReactMarkdown>

                  {/* Source Citations */}
                  {message.sources && message.sources.length > 0 && (
                    <div className={styles.sources}>
                      <div className={styles.sourcesTitle}>
                        <BookOpen size={14} />
                        <span>Sources:</span>
                      </div>
                      {message.sources.map((source, idx) => (
                        <div key={idx} className={styles.sourceItem}>
                          <span className={styles.sourceChapter}>
                            Chapter {source.chapter}, Lesson {source.lesson}
                          </span>
                          <span className={styles.sourceSection}>{source.section_title}</span>
                          <span className={styles.sourceScore}>
                            {(source.similarity_score * 100).toFixed(0)}% match
                          </span>
                        </div>
                      ))}
                    </div>
                  )}
                </div>
                <div className={styles.messageTime}>
                  {message.timestamp.toLocaleTimeString([], {
                    hour: '2-digit',
                    minute: '2-digit',
                  })}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageContent}>
                  <div className={styles.loadingIndicator}>
                    <Loader2 size={16} className={styles.spinner} />
                    <span>Thinking...</span>
                  </div>
                </div>
              </div>
            )}

            {error && (
              <div className={styles.errorMessage}>
                <span>⚠️ {error}</span>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Selected Text Indicator */}
          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <div className={styles.selectedTextContent}>
                <Sparkles size={16} />
                <span className={styles.selectedTextLabel}>Selected text:</span>
                <span className={styles.selectedTextPreview}>
                  "{selectedText.substring(0, 80)}{selectedText.length > 80 ? '...' : ''}"
                </span>
                <button
                  className={styles.clearSelectionButton}
                  onClick={() => {
                    setSelectedText(null);
                    clearSelection();
                  }}
                  title="Clear selection"
                >
                  <X size={14} />
                </button>
              </div>
            </div>
          )}

          {/* Input */}
          <div className={styles.inputContainer}>
            <input
              ref={inputRef}
              type="text"
              className={styles.input}
              placeholder={selectedText ? "Ask about the selected text..." : "Ask a question..."}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              disabled={isLoading}
            />
            <button
              className={styles.sendButton}
              onClick={handleSendMessage}
              disabled={isLoading || !inputValue.trim()}
              aria-label="Send message"
            >
              {isLoading ? <Loader2 size={20} className={styles.spinner} /> : <Send size={20} />}
            </button>
          </div>
        </div>
      )}
    </>
  );
}
