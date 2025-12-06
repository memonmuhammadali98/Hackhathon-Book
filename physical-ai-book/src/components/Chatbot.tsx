import React, { useState, useRef, useEffect } from 'react';
import styles from './Chatbot.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    text: string;
    metadata: {
      chapter?: string;
      title?: string;
      heading?: string;
    };
  }>;
}

interface ChatbotProps {
  apiUrl?: string;
}

const Chatbot: React.FC<ChatbotProps> = ({ apiUrl = 'http://localhost:8000' }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 10) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const queryRAG = async (userQuery: string, useSelectedText: boolean = false): Promise<any> => {
    const query = useSelectedText && selectedText
      ? `Based on this context: "${selectedText}"\n\nQuestion: ${userQuery}`
      : userQuery;

    const response = await fetch(`${apiUrl}/query`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        query: query,
        top_k: 5
      })
    });

    if (!response.ok) {
      throw new Error('Failed to query RAG backend');
    }

    return response.json();
  };

  const handleSend = async (useSelectedText: boolean = false) => {
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      role: 'user',
      content: input
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Query the RAG backend
      const ragResults = await queryRAG(input, useSelectedText);

      // Build context from results
      const context = ragResults.results
        .map((r: any, i: number) => `[${i + 1}] ${r.text}`)
        .join('\n\n');

      // Create response with sources
      const assistantMessage: Message = {
        role: 'assistant',
        content: `Based on the textbook content:\n\n${context}\n\nTo answer your question: ${input}\n\nThe relevant sections discuss ${ragResults.results[0]?.metadata?.heading || 'the topic'}.`,
        sources: ragResults.results
      };

      setMessages(prev => [...prev, assistantMessage]);

      // Clear selected text after using it
      if (useSelectedText) {
        setSelectedText('');
      }
    } catch (error) {
      console.error('Error querying RAG:', error);
      const errorMessage: Message = {
        role: 'assistant',
        content: 'Sorry, I encountered an error while searching the textbook. Please make sure the RAG backend is running.'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <>
      {/* Floating Chat Button */}
      {!isOpen && (
        <button
          className={styles.chatButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          💬
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3>📚 Ask about the Book</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Hi! I can help you understand the Physical AI & Humanoid Robotics textbook.</p>
                <p>Ask me anything about:</p>
                <ul>
                  <li>Physical AI concepts</li>
                  <li>Embodied intelligence</li>
                  <li>ROS 2 and robotics frameworks</li>
                  <li>VSLAM and Navigation</li>
                  <li>Simulation environments</li>
                </ul>
                {selectedText && (
                  <div className={styles.selectedTextHint}>
                    💡 You have text selected. Click "Ask about Selection" to query it!
                  </div>
                )}
              </div>
            )}

            {messages.map((msg, idx) => (
              <div key={idx} className={`${styles.message} ${styles[msg.role]}`}>
                <div className={styles.messageContent}>
                  {msg.content}
                </div>
                {msg.sources && msg.sources.length > 0 && (
                  <div className={styles.sources}>
                    <strong>📖 Sources:</strong>
                    <ul>
                      {msg.sources.map((source, i) => (
                        <li key={i}>
                          {source.metadata.chapter && `${source.metadata.chapter} - `}
                          {source.metadata.heading || source.metadata.title}
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.loadingDots}>
                  <span>.</span><span>.</span><span>.</span>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div className={styles.inputArea}>
            {selectedText && (
              <div className={styles.selectedTextBadge}>
                ✂️ Text selected ({selectedText.length} chars)
                <button onClick={() => setSelectedText('')}>✕</button>
              </div>
            )}
            <div className={styles.inputRow}>
              <textarea
                className={styles.input}
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask a question about the book..."
                rows={2}
                disabled={isLoading}
              />
              <div className={styles.buttonGroup}>
                <button
                  className={styles.sendButton}
                  onClick={() => handleSend(false)}
                  disabled={isLoading || !input.trim()}
                  title="Ask about whole book"
                >
                  📚 Ask
                </button>
                {selectedText && (
                  <button
                    className={`${styles.sendButton} ${styles.selectionButton}`}
                    onClick={() => handleSend(true)}
                    disabled={isLoading || !input.trim()}
                    title="Ask about selected text"
                  >
                    ✂️ Ask Selection
                  </button>
                )}
              </div>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default Chatbot;
