# Text Selection Feature - Chatbot Integration

## ✅ Feature Implemented

When you select text in the book, the chatbot automatically:
1. Opens (if closed)
2. Shows selected text in a banner
3. Pre-fills input with "Guide me about this content"
4. Uses selected text query endpoint for context-aware responses

## 🎯 How It Works

### User Flow:

1. **Select Text**: User selects any text in the book content
2. **Auto-Open**: Chatbot window automatically opens
3. **Show Selection**: Selected text appears in a banner above input
4. **Ask Question**: User can ask questions about the selected text
5. **Context-Aware**: Backend uses `/api/chat/selected-text` endpoint

### Technical Flow:

```
User selects text
    ↓
useTextSelection hook detects selection
    ↓
handleTextSelection callback triggered
    ↓
Chatbot opens + Selected text stored
    ↓
User asks question
    ↓
POST /api/chat/selected-text (with selected text)
    ↓
Backend RAG with selected text context
    ↓
Response with citations
```

## 📝 Usage

### For Users:

1. **Select any text** in the book content
2. Chatbot **automatically opens**
3. See selected text in the **banner above input**
4. Type your question (e.g., "Explain this", "What does this mean?")
5. Get **context-aware response** based on selected text

### Example:

```
User selects: "ROS 2 uses DDS for communication"
Chatbot opens automatically
Input pre-filled: "Guide me about this content"
User types: "Explain DDS"
Response: Context-aware explanation about DDS in ROS 2
```

## 🔧 Implementation Details

### Files Modified:

1. **`src/hooks/useTextSelection.ts`** (NEW)
   - Detects text selection globally
   - Returns selected text and position
   - Clears selection on demand

2. **`src/components/RAGChatbot/RAGChatbot.tsx`**
   - Integrated text selection hook
   - Auto-opens on selection
   - Shows selected text banner
   - Uses `sendSelectedTextQuery` API

3. **`src/components/RAGChatbot/styles.module.css`**
   - Added `.selectedTextBanner` styles
   - Added `.selectedTextContent` styles
   - Added `.clearSelectionButton` styles

### API Endpoint:

- **Endpoint**: `POST /api/chat/selected-text`
- **Request**:
  ```json
  {
    "session_id": "session-123",
    "message": "Explain this",
    "selected_text": "ROS 2 uses DDS..."
  }
  ```
- **Response**: Same as regular chat (with context from selected text)

## 🎨 UI Features

### Selected Text Banner:

- **Location**: Above input field in chatbot
- **Shows**: Selected text preview (first 80 chars)
- **Actions**: Clear button to remove selection
- **Style**: Gradient background with primary color

### Auto-Open Behavior:

- Chatbot opens automatically when text is selected
- Only opens if currently closed
- Maintains existing state if already open

## 🧪 Testing

1. **Select text** in any lesson page
2. Verify chatbot **opens automatically**
3. Check **selected text banner** appears
4. Type question and **send**
5. Verify response is **context-aware**

## 📚 Backend Integration

The backend already has the `/api/chat/selected-text` endpoint that:
- Takes selected text as context
- Searches Qdrant for related content
- Generates response using OpenAI Agents SDK
- Returns citations

No backend changes needed! ✅

## 🎯 Benefits

- **Better UX**: No need to manually copy-paste text
- **Context-Aware**: Responses based on selected content
- **Seamless**: Automatic chatbot opening
- **Visual Feedback**: Selected text shown in banner

---

**Status**: ✅ Fully Implemented and Ready to Use!

